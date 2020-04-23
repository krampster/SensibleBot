using System.Numerics;
using System.Windows.Media;
using Bot.Utilities.Processed.BallPrediction;
using Bot.Utilities.Processed.FieldInfo;
using Bot.Utilities.Processed.Packet;
using RLBotDotNet;
using System;
using System.Collections.Generic;

namespace Bot
{
    // We want to our bot to derive from Bot, and then implement its abstract methods.
    class Bot : RLBotDotNet.Bot
    {
        const float BallRadius = 91.25f;

        //SendQuickChatFromAgent()

        // We want the constructor for our Bot to extend from RLBotDotNet.Bot, but we don't want to add anything to it.
        // You might want to add logging initialisation or other types of setup up here before the bot starts.
        public Bot(string botName, int botTeam, int botIndex) : base(botName, botTeam, botIndex) { }

        public enum Tactic
        {
            Kickoff,
            Attack,
            Patience,
            Defend
        }

        public enum Mechanic
        {
            None,
            DriveToBall,
            WallPlay,
            Patience,
            Defend,
            Jump
        }

        public class CarState
        {
            public float distanceToBall;
            public int boost;
            public double pitch;
            public double roll;
            public double yaw;
            public Vector3 pos;
            public Vector3 velocity;
            public Vector3 forward;
        }

        public class MyCarState : CarState
        {
            public Tactic tactic;
            //public string location;
            public float distanceToOwnGoal;
            public float distanceToOppGoal;
            public double angleToTarget;
            public Vector3 target;
            public Vector3 closestBoost;
        };

        public class FieldState
        {
            public Vector3 ballVelocity;
            public Vector3 ballPos;
            public Vector3 oppGoal;
            public Vector3 ownGoal;
            public List<Vector3> boost;
        }

        public class GameState
        {
            public Physics BallPhysics;
            MyCarState MyCar;
            CarState OtherCar;
            FieldInfo Field;
            BallPrediction BallPrediction;
        }

        ////BotCalculations
        //Tactic
        //PrimaryMechanic
        //SecondaryMechanic
        //PrimaryTarget
        //PrimaryAngle
        //SecondaryTarget
        //SecondaryAngle
        //Controller

        public override Controller GetOutput(rlbot.flat.GameTickPacket gameTickPacket)
        {
            // We process the gameTickPacket and convert it to our own internal data structure.
            Packet packet = new Packet(gameTickPacket);
            Controller result = new Controller();

            if (packet.Players.Length == 0)
                return result;

            // Get the data required to drive to the ball.
            Vector3 ballActualLocation = packet.Ball.Physics.Location;
            Vector3 carLocation = packet.Players[index].Physics.Location;
            Orientation carRotation = packet.Players[index].Physics.Rotation;

            // We normally want to hit the ball on OUR side.
            Vector3 myGoalLocation = GetFieldInfo().Goals[packet.Players[index].Team].Location;
            Vector3 otherGoalLocation = GetFieldInfo().Goals[Math.Abs(packet.Players[index].Team - 1)].Location;

            float distanceToBall = DistanceBetween(carLocation, ballActualLocation);
            BallPrediction prediction = GetBallPrediction();
            int predictionSlice = (int)distanceToBall / 25 - 1;
            predictionSlice = (int)Clamp(predictionSlice, 0, prediction.Length - 1);
            Vector3 ballLocation = prediction.Slices[predictionSlice].Physics.Location;

            // How far away from the ball should we be aiming?
            //
            Vector3 ballEdgeLocation = ProjectLocationTowardsTarget(ballLocation, otherGoalLocation, -BallRadius * 1.25f);
            distanceToBall = DistanceBetween(carLocation, ballEdgeLocation);

            float distanceToBallToAim = BallRadius + .25f * distanceToBall;
            Vector3 targetLocation = ProjectLocationTowardsTarget(ballLocation, otherGoalLocation, -distanceToBallToAim);

            // Find where the ball is relative to us.
            Vector3 ballRelativeLocation = Orientation.RelativeLocation(carLocation, targetLocation, carRotation);

            float steer;
            float throttle = 1;

            // Check the angle to the ball and steer that way.
            float angleToBallTarget = AngleBetween(carLocation, carRotation, targetLocation);

            // Specify steer strength as less when ball is far. (unnecessary?)
            const float maxTurnEasingDistance = 800;
            const float minTurnEasingDistance = 300;
            float distanceFactor = Clamp(distanceToBall, minTurnEasingDistance, maxTurnEasingDistance);
            //800 -- 4
            //300 -- 8
            float steerStrength = Lerp(4.0f, 8, (maxTurnEasingDistance - distanceFactor) / (maxTurnEasingDistance - minTurnEasingDistance));
            steer = Clamp(angleToBallTarget * steerStrength, -1, 1);

            if (Math.Abs(angleToBallTarget) < 0.2f)
            {
                result.Boost = true;
            }

            if (Math.Abs(angleToBallTarget) > 1.6f)
            {
                result.Handbrake = true;
            }

            // get off the wall
            if (carRotation.Roll > .5)
            {
                steer = 1;
                result.Roll = -1;
            }
            else if (carRotation.Roll < -.5)
            {
                steer = -1;
                result.Roll = 1;
            }
            

            //jump
            if (Math.Abs(ballRelativeLocation.X) < 250 && 
                Math.Abs(ballRelativeLocation.Y) < 150 &&
                ballLocation.Z > 100 && ballLocation.Z < 300)
            {
                result.Jump = true;
            }
            else if (ballLocation.Z > 300)
            {
                //patience
                Vector3 safePosition = ProjectLocationTowardsTarget(myGoalLocation, otherGoalLocation, 500);
                float angleToSafePosition = AngleBetween(carLocation, carRotation, safePosition);

                if (Math.Abs(angleToSafePosition) > Math.PI * 0.5f)
                {
                    //Reverse to safe position
                    throttle = -1;
                    float reverseAngle;
                    if (angleToSafePosition > 0)
                    {
                        reverseAngle = (float)Math.PI - angleToSafePosition;
                    }
                    else
                    {
                        reverseAngle = -(float)Math.PI + angleToSafePosition;
                    }
                    steer = Clamp(reverseAngle, -1, 1);
                }
                else
                {
                    //turn to safe position
                    throttle = 0.2f;
                    steer = Clamp(angleToSafePosition, -1, 1);
                }
            }

            // Examples of rendering in the game
            Renderer.DrawString3D("Ball", Colors.Black, ballEdgeLocation, 3, 3);
            if (steer != 0)
            {
                Renderer.DrawString3D(steer > 0 ? "Right" : "Left", Colors.Aqua, carLocation, 3, 3);
            }
            if (result.Jump)
            {
                Renderer.DrawString3D("Jump", Colors.Green, carLocation, 3, 3);
            }

            Renderer.DrawLine3D(Colors.Red, carLocation, ballEdgeLocation);

            // This controller will contain all the inputs that we want the bot to perform.
            result.Throttle = throttle;
            result.Steer = steer;
            return result;
        }

        private bool IsPointingAtGoal(Vector3 location, Orientation orientation, int team)
        {
            bool result = false;

            float angle = AngleBetween(location, orientation, GetFieldInfo().Goals[team].Location);
            result = Math.Abs(angle) < Math.PI / 4;
            return result;
        }

        private float DistanceBetween(Vector3 location1, Vector3 location2)
        {
            return (location1 - location2).Length();
        }

        private float AngleBetween(Vector3 referenceLocation, Orientation referenceOrientation, Vector3 targetLocation)
        {
            Vector3 relativelocation = Orientation.RelativeLocation(referenceLocation, targetLocation, referenceOrientation);

            Double angle = Math.Atan2(relativelocation.Y, relativelocation.X);
            return (float)angle;
        }

        private Vector3 ProjectLocationTowardsTarget(Vector3 referenceLocation, Vector3 targetLocation, float distance)
        {
            Vector3 delta = targetLocation - referenceLocation;
            delta = Vector3.Normalize(delta);
            return referenceLocation + (delta * distance);            
        }

        float Lerp(float firstFloat, float secondFloat, float by)
        {
            return firstFloat * (1 - by) + secondFloat * by;
        }

        public static float Clamp(float value, float min, float max)
        {
            return (value < min) ? min : (value > max) ? max : value;
        }

        public double getDistance2D(double x1, double x2, double y1, double y2)
        {
            return Math.Sqrt(Math.Pow((x2 - x1), 2) + Math.Pow((y2 - y1), 2));
        }

        public double getDistance2D(Vector3 pointA, Vector3 pointB)
        {
            return getDistance2D(pointA.X, pointB.X, pointA.Y, pointB.Y);
        }

        //public double magnitude2D(Vector3 vector)
        //{
        //    return Math.Sqrt(Math.Pow((vector.X - vector.X), 2) + Math.Pow((vector.Y - vector.Y), 2));
        //}


        // private Vector3 getBounceLocation(rlbot.flat.BallPrediction prediction)
        //{
        //     for (int i = 0; i < prediction.SlicesLength; i++)
        //     {
        //         Vector3 point = fromFramework(prediction.Slices(i).Value.Physics.Value.Location.Value);
        //         if (point.Z < 125)
        //         {
        //             renderPrediction(prediction, 0, i, System.Windows.Media.Color.FromRgb(255, 0, 255));
        //             return point;
        //         }
        //     }
        //     return fromFramework(prediction.Slices(0).Value.Physics.Value.Location.Value);
        // }

        //private Controller getDodgeOutput(Controller controller, double steer)
        //{
        //    if (dodgeWatch.ElapsedMilliseconds <= 120)
        //    {
        //        controller.Jump = (dodgeWatch.ElapsedMilliseconds <= 80);
        //        controller.Yaw = 0;
        //        controller.Pitch = 0;
        //    }
        //    else if (dodgeWatch.ElapsedMilliseconds <= 250)
        //    {
        //        controller.Jump = true;
        //        controller.Yaw = (float)-Math.Sin(steer);
        //        controller.Pitch = (float)-Math.Cos(steer);
        //    }
        //    else if (dodgeWatch.ElapsedMilliseconds <= 1000)
        //    {
        //        controller.Jump = false;
        //        controller.Yaw = 0;
        //        controller.Pitch = 0;
        //    }
        //    if (dodgeWatch.ElapsedMilliseconds <= 800) controller.Boost = false;

        //    return controller;
        //}


        //private Vector3 getHitPoint(rlbot.flat.GameTickPacket gameTickPacket, rlbot.flat.BallPrediction prediction)
        //{
        //    Vector3 carLocation = fromFramework(gameTickPacket.Players(this.index).Value.Physics.Value.Location.Value);
        //    double u = fromFramework(gameTickPacket.Players(this.index).Value.Physics.Value.Velocity.Value).Length();

        //    // Estimate the maximum velocity.           
        //    double maxV = Math.Max(1410, Math.Min(2300, u + 150 * gameTickPacket.Players(this.index).Value.Boost));

        //    for (int i = 0; i < prediction.SlicesLength; i++)
        //    {
        //        Vector3 point = fromFramework(prediction.Slices(i).Value.Physics.Value.Location.Value);

        //        double s = Vector3.Distance(point, carLocation) - 92.75;
        //        double t = (double)i / 60D;
        //        double v = 2D * (s / t) - u;
        //        double a = (Math.Pow(v, 2) - Math.Pow(u, 2)) / (2 * s);

        //        if (v <= maxV && a < 1700) // Approximate max acceleration.
        //        {
        //            renderPrediction(prediction, 0, i, System.Windows.Media.Color.FromRgb(255, 255, 255));
        //            return point;
        //        }
        //    }
        //    return fromFramework(prediction.Slices(0).Value.Physics.Value.Location.Value);
        //}

        private Vector3? getClosestBoost(rlbot.flat.GameTickPacket gameTickPacket, Vector3 carLocation)
        {
            Vector3? closest = null;
            double closestDistance = 0;
            for (int i = 0; i < gameTickPacket.BoostPadStatesLength; i++)
            {
                rlbot.flat.BoostPadState boostPadState = (rlbot.flat.BoostPadState)gameTickPacket.BoostPadStates(i);
                Vector3 boostPosition = GetFieldInfo().BoostPads[i].Location;
                double boostDistance = getDistance2D(carLocation.X, boostPosition.X, carLocation.Y, boostPosition.Y);

                if (boostPadState.IsActive && /*boostPosition.IsFullBoost && */(closest == null || closestDistance > boostDistance))
                {
                    closestDistance = boostDistance;
                    closest = boostPosition;
                }
            }
            return closest;
        }

        // Hide the old methods that return Flatbuffers objects and use our own methods that
        // use processed versions of those objects instead.
        internal new FieldInfo GetFieldInfo() => new FieldInfo(base.GetFieldInfo());
        internal new BallPrediction GetBallPrediction() => new BallPrediction(base.GetBallPrediction());
    }
}