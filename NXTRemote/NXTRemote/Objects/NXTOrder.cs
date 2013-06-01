using System;

namespace NXTRemote.Objects
{
    /// <summary>
    /// Represent an order.
    /// </summary>
    public class NXTOrder
    {

        private string p_order = null;
        private int p_effort = 0;
        private bool p_direction = true;

        public string Order
        {
            get { return p_order; }
            set { p_order = value; }
        }

        public int Effort
        {
            get { return p_effort; }
            set { p_effort = value; }
        }

        public bool Direction
        {
            get { return p_direction; }
            set { p_direction = value; }
        }

        public NXTOrder()
        {
        }

        public NXTOrder(string order, int effort, bool direction)
        {
            this.p_order = order;
            this.p_effort = effort;
            this.p_direction = direction;
        }

    }
}
