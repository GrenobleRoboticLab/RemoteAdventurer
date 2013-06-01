using System;

namespace NXTRemote.Objects
{
    /// <summary>
    /// Define the parts of the NXT to work with.
    /// </summary>
    public class NXTEntity
    {

        private NXTDashboard_Color p_color = new NXTDashboard_Color();
        private NXTDashboard_Contact p_leftContact = new NXTDashboard_Contact();
        private NXTDashboard_Contact p_rightContact = new NXTDashboard_Contact();
        private NXTDashboard_Ultrasonic p_ultrasonic = new NXTDashboard_Ultrasonic();
        private NXTDashboard_Wheel p_auxWheel = new NXTDashboard_Wheel();
        private NXTDashboard_Wheel p_leftWheel = new NXTDashboard_Wheel();
        private NXTDashboard_Wheel p_rightWheel = new NXTDashboard_Wheel();

        public NXTDashboard_Color Color
        {
            get { return p_color; }
            set { p_color = value; }
        }

        public NXTDashboard_Contact LeftContact
        {
            get { return p_leftContact; }
            set { p_leftContact = value; }
        }

        public NXTDashboard_Contact RightContact
        {
            get { return p_rightContact; }
            set { p_rightContact = value; }
        }

        public NXTDashboard_Ultrasonic Ultrasonic
        {
            get { return p_ultrasonic; }
            set { p_ultrasonic = value; }
        }

        public NXTDashboard_Wheel AuxWheel
        {
            get { return p_auxWheel; }
            set { p_auxWheel = value; }
        }

        public NXTDashboard_Wheel LeftWheel
        {
            get { return p_leftWheel; }
            set { p_leftWheel = value; }
        }

        public NXTDashboard_Wheel RightWheel
        {
            get { return p_rightWheel; }
            set { p_rightWheel = value; }
        }

        public NXTEntity()
        {
        }

        public NXTEntity(NXTDashboard_Color color, NXTDashboard_Contact leftContact, NXTDashboard_Contact rightContact, NXTDashboard_Ultrasonic ultrasonic, NXTDashboard_Wheel auxWheel, NXTDashboard_Wheel leftWheel, NXTDashboard_Wheel rightWheel)
        {
            this.p_color = color;
            this.p_leftContact = leftContact;
            this.p_rightContact = rightContact;
            this.p_ultrasonic = ultrasonic;
            this.p_auxWheel = auxWheel;
            this.p_leftWheel = leftWheel;
            this.p_rightWheel = rightWheel;
        }

    }
}
