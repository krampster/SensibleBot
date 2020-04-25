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
    public enum TacticType
    {
        Kickoff,
        Attack,
        Patience,
        Defend
    }

    public enum MechanicType
    {
        None,
        DriveToBall,
        WallPlay,
        Patience,
        Defend,
        Jump
    }

    public class MyCarState
    {
        public MyCarState(Player inPlayer)
        {
            player = inPlayer;
        }
        public Player player;
        public float distanceToOwnGoal;
        public float distanceToOppGoal;
        //public double angleToTarget;
        //public Vector3 closestBoost;
    };

    public class FieldState
    {
        public FieldInfo field;
        public Vector3 oppGoalLocation;
        public Vector3 ownGoalLocation;
        public List<Vector3> boost;
    }

    public class GameState
    {
        public Packet packet;
        public Physics ballPhysics;
        public MyCarState myCar;
        //public CarState otherCar;
        public FieldState fieldState;
        public BallPrediction ballPrediction;

        public Vector3 GetPredictionLocation()
        {
            // Use prediction based on how far we are from the ball.
            // TODO: reduce this by how far the opponent is from the ball.
            float distanceToBall = Utils.DistanceBetween(myCar.player.Physics.Location, ballPhysics.Location);
            int predictionSlice = (int)distanceToBall / 25 - 1;
            predictionSlice = (int)Utils.Clamp(predictionSlice, 0, ballPrediction.Length - 1);
            Vector3 ballLocation = ballPrediction.Slices[predictionSlice].Physics.Location;
            return ballLocation;
        }
    }

    public class BotCalculations
    {
        public Tactic tactic;
        //public Mechanic primaryMechanic;
        public Vector3 primaryTarget;
        public float primaryAngle;
        public Vector3 predictedBallLocation;
        public Controller controller;
    }


    public abstract class Tactic
    {
        public TacticType tacticType;

        public Tactic(TacticType type)
        {
            tacticType = type;
        }

        public abstract void DoWork(GameState gameState, BotCalculations botCalculations);

        public void ContactBall(GameState gamestate, BotCalculations botCalculations)
        {
            // Simple Jump.
            // Find where the ball is relative to us.
            Vector3 ballLocation = gamestate.ballPhysics.Location;
            Vector3 ballRelativeLocation = Orientation.RelativeLocation(gamestate.myCar.player.Physics.Location, ballLocation, gamestate.myCar.player.Physics.Rotation);

            if (Math.Abs(ballRelativeLocation.X) < 400 &&
                Math.Abs(ballRelativeLocation.Y) < 150 &&
                ballLocation.Z > 100 && ballLocation.Z < 300)
            {
                botCalculations.controller.Jump = true;
            }
        }

        public void AirWallRecovery(GameState gamestate, BotCalculations botCalculations)
        {
            // TODO: Fix up yaw and pitch.
            // get off the wall
            if (gamestate.myCar.player.Physics.Rotation.Roll > .5)
            {
                botCalculations.controller.Steer = 1;
                botCalculations.controller.Roll = -1;
            }
            else if (gamestate.myCar.player.Physics.Rotation.Roll < -.5)
            {
                botCalculations.controller.Steer = -1;
                botCalculations.controller.Roll = 1;
            }
        }
    }

    public class Attack : Tactic
    {
        public Attack() : base(TacticType.Attack)
        {
        }

        public override void DoWork(GameState gameState, BotCalculations botCalculations)
        {
            // Get the data required to drive to the ball.
            Vector3 ballActualLocation = gameState.ballPhysics.Location;
            Vector3 carLocation = gameState.myCar.player.Physics.Location;
            Orientation carRotation = gameState.myCar.player.Physics.Rotation;

            Vector3 ballLocation = gameState.GetPredictionLocation();
            botCalculations.predictedBallLocation = ballLocation;

            // Target the side of the ball to shoot at their goal.
            Vector3 ballEdgeLocation = Utils.ProjectLocationTowardsTarget(ballLocation, gameState.fieldState.oppGoalLocation, -Utils.BallRadius * 1.25f);
            float distanceToBall = Utils.DistanceBetween(carLocation, ballEdgeLocation);

            // How far away from the ball should we be aiming?
            float glidepathAimAmount = Math.Min(.25f * distanceToBall * distanceToBall / 1000.0f, 800);
            float distanceToBallToAim = Utils.BallRadius + glidepathAimAmount;
            Vector3 targetLocation = Utils.ProjectLocationTowardsTarget(ballLocation, gameState.fieldState.oppGoalLocation, -distanceToBallToAim);
            targetLocation = Utils.ClampToField(targetLocation);
            botCalculations.primaryTarget = targetLocation;

            // Check the angle to the ball and steer that way.
            float angleToBallTarget = Utils.AngleBetween(carLocation, carRotation, targetLocation);
            botCalculations.primaryAngle = angleToBallTarget;

            // Specify steer strength as less when ball is far. (unnecessary?)
            const float minTurnEasingDistance = 300;
            const float maxTurnEasingDistance = 1000;
            float distanceFactor = Utils.Clamp(distanceToBall, minTurnEasingDistance, maxTurnEasingDistance);
            float steerStrength = Utils.Lerp(6.0f, 10, (maxTurnEasingDistance - distanceFactor) / (maxTurnEasingDistance - minTurnEasingDistance));

            // Steer stronger when moving fast.
            if (gameState.myCar.player.Physics.Velocity.Length() > 1000)
            {
                steerStrength *= 2;
            }

            botCalculations.controller.Steer = Utils.Clamp(angleToBallTarget * steerStrength, -1, 1);

            if (Math.Abs(angleToBallTarget) < 0.2f)
            {
                // TODO: don't boost when unnecessary. Both for worthless hits, and lack of urgency?
                botCalculations.controller.Boost = true;
            }
            botCalculations.controller.Throttle = 1;


            if (Math.Abs(angleToBallTarget) > 1.6f)
            {
                botCalculations.controller.Handbrake = true;
            }
        
            // Fixups.
            AirWallRecovery(gameState, botCalculations);

            // Jump.
            ContactBall(gameState, botCalculations);
        }
    }

    public class Patience : Tactic
    {
        public Patience() : base(TacticType.Patience)
        {
        }

        public override void DoWork(GameState gameState, BotCalculations botCalculations)
        {
            Vector3 safePosition = Utils.ProjectLocationTowardsTarget(gameState.fieldState.ownGoalLocation, gameState.fieldState.oppGoalLocation, 300);
            float angleToSafePosition = Utils.AngleBetween(gameState.myCar.player.Physics.Location, gameState.myCar.player.Physics.Rotation, safePosition);
            botCalculations.primaryTarget = safePosition;
            botCalculations.primaryAngle = angleToSafePosition;

            if (Math.Abs(angleToSafePosition) > Math.PI * 0.5f)
            {
                // Reverse to safe position.
                botCalculations.controller.Throttle = -1;
                float reverseAngle;
                if (angleToSafePosition > 0)
                {
                    reverseAngle = (float)Math.PI - angleToSafePosition;
                }
                else
                {
                    reverseAngle = -(float)Math.PI + angleToSafePosition;
                }
                botCalculations.controller.Steer = Utils.Clamp(reverseAngle, -1, 1);
            }
            else
            {
                // Turn to safe position.
                botCalculations.controller.Throttle = 0.2f;
                botCalculations.controller.Steer = Utils.Clamp(angleToSafePosition, -1, 1);
            }

            // Fixups.
            AirWallRecovery(gameState, botCalculations);

            // Jump.
            ContactBall(gameState, botCalculations);
        }
    }

    // We want to our bot to derive from Bot, and then implement its abstract methods.
    public class Bot : RLBotDotNet.Bot
    {
        Tactic currentTactic;
        //Mechanic currentMechanic;

        //SendQuickChatFromAgent()

        // We want the constructor for our Bot to extend from RLBotDotNet.Bot, but we don't want to add anything to it.
        // You might want to add logging initialisation or other types of setup up here before the bot starts.
        public Bot(string botName, int botTeam, int botIndex) : base(botName, botTeam, botIndex) { }



        public GameState InitializeGameState(rlbot.flat.GameTickPacket gameTickPacket)
        {
            GameState gameState = new GameState();
            gameState.packet = new Packet(gameTickPacket);
            gameState.ballPhysics = gameState.packet.Ball.Physics;

            if (gameState.packet.Players.Length == 0)
                return null;

            gameState.myCar = new MyCarState(gameState.packet.Players[index]);
            gameState.fieldState = new FieldState();
            gameState.fieldState.field = GetFieldInfo();

            gameState.fieldState.ownGoalLocation = gameState.fieldState.field.Goals[gameState.myCar.player.Team].Location;
            gameState.fieldState.oppGoalLocation = gameState.fieldState.field.Goals[Math.Abs(gameState.myCar.player.Team - 1)].Location;
            gameState.fieldState.ownGoalLocation.Z = 0;
            gameState.fieldState.oppGoalLocation.Z = 0;
            gameState.ballPrediction = GetBallPrediction();

            gameState.myCar.distanceToOwnGoal = Utils.DistanceBetween(gameState.myCar.player.Physics.Location, gameState.fieldState.ownGoalLocation);
            gameState.myCar.distanceToOppGoal = Utils.DistanceBetween(gameState.myCar.player.Physics.Location, gameState.fieldState.oppGoalLocation);

            return gameState;
        }

        public Tactic ChooseTactic(GameState gameState, BotCalculations botCalculations)
        {
            if (botCalculations.predictedBallLocation.Z > 300)
            {
                return new Patience();
            }
            else
            {
                return new Attack();
            }
        }

        public override Controller GetOutput(rlbot.flat.GameTickPacket gameTickPacket)
        {
            // Convert the current frame data into our own structures and helpers.
            GameState gameState = InitializeGameState(gameTickPacket);
            if (gameState == null)
            {
                return new Controller();
            }

            BotCalculations botCalculations = new BotCalculations
            {
                controller = new Controller()
            };
            botCalculations.predictedBallLocation = gameState.GetPredictionLocation();

            // Decide on a tactic and run it.
            currentTactic = ChooseTactic(gameState, botCalculations);
            currentTactic.DoWork(gameState, botCalculations);

            // Examples of rendering in the game
            Renderer.DrawString3D("Ball", Colors.Black, botCalculations.predictedBallLocation, 3, 3);
            if (botCalculations.controller.Steer != 0)
            {
                Renderer.DrawString3D(botCalculations.controller.Steer > 0 ? "Right" : "Left", Colors.Aqua, gameState.myCar.player.Physics.Location, 3, 3);
            }
            if (botCalculations.controller.Jump)
            {
                Renderer.DrawString3D("Jump", Colors.Green, gameState.myCar.player.Physics.Location, 3, 3);
            }

            Renderer.DrawLine3D(Colors.Red, gameState.myCar.player.Physics.Location, botCalculations.primaryTarget);


            return botCalculations.controller;
        }

        private bool IsPointingAtGoal(Vector3 location, Orientation orientation, int team)
        {
            bool result = false;

            float angle = Utils.AngleBetween(location, orientation, GetFieldInfo().Goals[team].Location);
            result = Math.Abs(angle) < Math.PI / 4;
            return result;
        }

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
                double boostDistance = Utils.getDistance2D(carLocation.X, boostPosition.X, carLocation.Y, boostPosition.Y);

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