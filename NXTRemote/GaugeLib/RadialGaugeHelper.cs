using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace GaugeLib
{
    public class RadialGaugeHelper
    {
        public static Point GetCenterPosition(RadialType type, Size finalSize, double minAngle, double maxAngle, SweepDirection sweepDir)
        {
            //get the quadrants of the limits
            int q1 = GetQuadrant(minAngle);
            int q2 = GetQuadrant(maxAngle);
            if (sweepDir == SweepDirection.Counterclockwise)
            {
                q1++; q2++;
                if (q1 > 4) q1 = 1;
                if (q2 > 4) q2 = 1;
            }
            else
            {
                //q2->q4 and q4->q2
                if (q1 % 2 == 0) q1 = 6 - q1;
                if (q2 % 2 == 0) q2 = 6 - q2;
            }
            //calculate the difference
            int diff = q2 - q1;
            if (Math.Abs(diff) == 0)
            {//quarter possibility
                if (type == RadialType.Quadrant)
                {
                    return GetCenterForQuadrant(q2, finalSize);
                }
                else if (type == RadialType.Semicircle)
                {
                    if (q1 == 1 || q1 == 2)
                        return new Point(finalSize.Width / 2, finalSize.Height);
                    else
                        return new Point(finalSize.Width / 2, 0);
                }
                else
                {//full circle
                    return new Point(finalSize.Width / 2, finalSize.Height / 2);
                }
            }
            else if (Math.Abs(diff) == 1 || (Math.Abs(diff)==3 && (maxAngle-minAngle)<=180))
            {//semicircle possibility
                if (type == RadialType.Quadrant || type == RadialType.Semicircle)
                {
                    return GetCenterForSemicircle(q1, q2, finalSize);
                }
                else
                {//full circle
                    return new Point(finalSize.Width / 2, finalSize.Height / 2);
                }
            }
            else
            {//full circle
                return new Point(finalSize.Width / 2, finalSize.Height / 2);
            }

        }
        public static double GetRadius(RadialType type, Size finalSize, double minAngle, double maxAngle, SweepDirection sweepDir)
        {
            
            //get the quadrants of the limits
            int q1 = GetQuadrant(minAngle);
            int q2 = GetQuadrant(maxAngle);
            if (sweepDir == SweepDirection.Counterclockwise)
            {
                q1++; q2++;
                if (q1 > 4) q1 = 1;
                if (q2 > 4) q2 = 1;
            }
            else
            {
                //q2->q4 and q4->q2
                if (q1 % 2 == 0) q1 = 6 - q1;
                if (q2 % 2 == 0) q2 = 6 - q2;
            }
            //calculate the difference
            int diff = q2 - q1;
            if (Math.Abs(diff) == 0)
            {//quarter possibility
                if (type == RadialType.Quadrant)
                {
                    return Math.Min(finalSize.Width, finalSize.Height);
                }
                else if (type == RadialType.Semicircle)
                {
                    return finalSize.Height;
                }
                else
                {//full circle
                    return Math.Min(finalSize.Width / 2, finalSize.Height / 2);
                }
            }
            else if (Math.Abs(diff) == 1 || (Math.Abs(diff) == 3 && (maxAngle - minAngle) <= 180))
            {//semicircle possibility
                if (type == RadialType.Quadrant || type == RadialType.Semicircle)
                {
                    return GetRadiusForSemicircle(q1, q2, finalSize);
                }
                else
                {//full circle
                    return Math.Min(finalSize.Width / 2, finalSize.Height / 2);
                }
            }
            else
            {//full circle
                return Math.Min(finalSize.Width / 2, finalSize.Height / 2);
            }
        }

        #region helper methods 
        private static double GetRadiusForSemicircle(int q1, int q2, Size finalSize)
        {
            if (q1 == 1 && q2 == 2 || q2 == 1 && q1 == 2)
            {
                return Math.Min(finalSize.Width/2, finalSize.Height );
            }
            else if (q1 == 2 && q2 == 3 || q1 == 3 && q2 == 2)
            {
                return Math.Min(finalSize.Width , finalSize.Height/2);
            }
            else if (q1 == 3 && q2 == 4 || q2 == 3 && q1 == 4)
            {
                return Math.Min(finalSize.Width / 2, finalSize.Height);
            }
            else
            {
                return Math.Min(finalSize.Width, finalSize.Height / 2);
            }
        }
        private static Point GetCenterForQuadrant(int q, Size finalSize)
        {
            if (q == 1)
                return new Point(0, finalSize.Height);
            else if (q == 2)
                return new Point(finalSize.Width, finalSize.Height);
            else if (q == 3)
                return new Point(finalSize.Width, 0);
            else
                return new Point(0, 0);
        }
        private static Point GetCenterForSemicircle(int q1, int q2, Size finalSize)
        {
            if (q1 == 1 && q2 == 2 || q2 == 1 && q1 == 2)
                return new Point(finalSize.Width/2, finalSize.Height);
            else if (q1 == 2 && q2 == 3 || q1 == 3 && q2 == 2)
                return new Point(finalSize.Width, finalSize.Height/2);
            else if (q1 == 3 && q2 == 4 || q1 == 4 && q2 == 3)
                return new Point(finalSize.Width/2, 0);
            else
                return new Point(0, finalSize.Height / 2);
        }
        private static int GetQuadrant(double angle)
        {
            angle = angle * Math.PI / 180;
            if (Math.Sin(angle) >= 0 && Math.Cos(angle) >= 0)
                return 1;
            else if (Math.Sin(angle) >= 0 && Math.Cos(angle) < 0)
                return 2;
            else if (Math.Sin(angle) < 0 && Math.Cos(angle) < 0)
                return 3;
            else
                return 4;
        }
        #endregion
    }
}
