using System;
using System.Numerics;
using Bot.Utilities.Processed.BallPrediction;
using Bot.Utilities.Processed.FieldInfo;
using Bot.Utilities.Processed.Packet;
using RLBotDotNet;

namespace Bot
{
    public static class Utils
    {
        public const float BallRadius = 91.25f;
        public const float SideWall = 4096;
        public const float BackWall = 5120;
        public const float GoalPostOffset = 850;
        public const float RadToDegrees = (float)180.0f / (float)Math.PI;
        public const float MaxSpeed = 1410;

        public static float DistanceBetween(Vector3 location1, Vector3 location2)
        {
            return (location1 - location2).Length();
        }

        public static float DistanceBetween2D(Vector3 location1, Vector3 location2)
        {
            location1.Z = 0;
            location2.Z = 0;
            return (location1 - location2).Length();
        }


        public static float AngleBetween(Vector3 referenceLocation, Orientation referenceOrientation, Vector3 targetLocation)
        {
            Vector3 relativelocation = Orientation.RelativeLocation(referenceLocation, targetLocation, referenceOrientation);

            Double angle = Math.Atan2(relativelocation.Y, relativelocation.X);
            return (float)angle;
        }

        public static Vector3 ProjectLocationTowardsTarget(Vector3 referenceLocation, Vector3 targetLocation, float distance)
        {
            Vector3 delta = targetLocation - referenceLocation;
            delta = Vector3.Normalize(delta);
            return referenceLocation + (delta * distance);
        }

        public static float Lerp(float firstFloat, float secondFloat, float by)
        {
            return firstFloat * (1 - by) + secondFloat * by;
        }

        public static float InverseLerp(float firstFloat, float secondFloat, float value)
        {
            value = Clamp(value, firstFloat, secondFloat);
            return (value - firstFloat) / (secondFloat - firstFloat);
        }

        public static float Clamp(float value, float min, float max)
        {
            return (value < min) ? min : (value > max) ? max : value;
        }

        public static double getDistance2D(double x1, double x2, double y1, double y2)
        {
            return Math.Sqrt(Math.Pow((x2 - x1), 2) + Math.Pow((y2 - y1), 2));
        }

        public static double getDistance2D(Vector3 pointA, Vector3 pointB)
        {
            return getDistance2D(pointA.X, pointB.X, pointA.Y, pointB.Y);
        }

        public static Vector3 ClampToField(Vector3 value)
        {
            Vector3 result = value;
            result.X = Clamp(value.X, -SideWall, SideWall);
            result.Y = Clamp(value.Y, -BackWall, BackWall);
            return result;
        }
    }
}
