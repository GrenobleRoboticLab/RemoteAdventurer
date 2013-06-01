using System;

namespace NXTRemote.Objects
{
    /// <summary>
    /// Represent a color captor.
    /// </summary>
    public class NXTDashboard_Color
    {
        private int p_intensity = 0;
        private int p_red = 0;
        private int p_green = 0;
        private int p_blue = 0;

        public int Intensity
        {
            get { return p_intensity; }
            set { p_intensity = value; }
        }

        public int Red
        {
            get { return p_red; }
            set { p_red = value; }
        }

        public int Green
        {
            get { return p_green; }
            set { p_green = value; }
        }

        public int Blue
        {
            get { return p_blue; }
            set { p_blue = value; }
        }

        public NXTDashboard_Color()
        {
        }

        public NXTDashboard_Color(int intensity, int red, int green, int blue)
        {
            this.p_intensity = intensity;
            this.p_red = red;
            this.p_green = green;
            this.p_blue = blue;
        }
    }

    /// <summary>
    /// Represent a touch captor.
    /// </summary>
    public class NXTDashboard_Contact
    {
        private bool p_isTouching = false;

        public bool IsTouching
        {
            get { return p_isTouching; }
            set { p_isTouching = value; }
        }

        public NXTDashboard_Contact()
        {
        }

        public NXTDashboard_Contact(bool isTouching)
        {
            this.p_isTouching = isTouching;
        }
    }

    /// <summary>
    /// Represent an ultrasonic captor.
    /// </summary>
    public class NXTDashboard_Ultrasonic
    {
        private int p_range = 0;
        private int p_rangeMin = 0;
        private int p_rangeMax = 0;
        private int p_spreadAngle = 0;

        public int Range
        {
            get { return p_range; }
            set { p_range = value; }
        }

        public int RangeMin
        {
            get { return p_rangeMin; }
            set { p_rangeMin = value; }
        }

        public int RangeMax
        {
            get { return p_rangeMax; }
            set { p_rangeMax = value; }
        }

        public int SpreadAngle
        {
            get { return p_spreadAngle; }
            set { p_spreadAngle = value; }
        }

        public NXTDashboard_Ultrasonic()
        {
        }

        public NXTDashboard_Ultrasonic(int range, int rangeMin, int rangeMax, int spreadAngle)
        {
            this.p_range = range;
            this.p_rangeMin = rangeMin;
            this.p_rangeMax = rangeMax;
            this.p_spreadAngle = spreadAngle;
        }
    }

    /// <summary>
    /// Represent a wheel engine.
    /// </summary>
    public class NXTDashboard_Wheel
    {
        private string p_name = null;
        private int p_effort = 0;
        private int p_position = 0;
        private int p_velocity = 0;

        public string Name
        {
            get { return p_name; }
            set { p_name = value; }
        }

        public int Effort
        {
            get { return p_effort; }
            set { p_effort = value; }
        }

        public int Position
        {
            get { return p_position; }
            set { p_position = value; }
        }

        public int Velocity
        {
            get { return p_velocity; }
            set { p_velocity = value; }
        }

        public NXTDashboard_Wheel()
        {
        }

        public NXTDashboard_Wheel(string name, int effort, int position, int velocity)
        {
            this.p_name = name;
            this.p_effort = effort;
            this.p_position = position;
            this.p_velocity = velocity;
        }
    }

}