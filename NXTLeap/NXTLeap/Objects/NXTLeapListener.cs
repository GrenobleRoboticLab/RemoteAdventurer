using System;

namespace NXTLeap.Objects
{
    public class NXTLeapListener : Leap.Listener
    {
        public delegate void DelegateLeapLog(string log);
        public delegate void DelegateLeapHand(float pn_x, float pn_y, float pn_z, float pp_x, float pp_y, float pp_z, float pd_x, float pd_y, float pd_z);
        public delegate void DelegateLeapCircle(int circleId, string direction);

        public event DelegateLeapLog Initialized;
        public event DelegateLeapLog Connected;
        public event DelegateLeapLog Disconnected;
        public event DelegateLeapLog Exited;
        public event DelegateLeapLog Framed;
        public event DelegateLeapHand ReturnHand;
        public event DelegateLeapCircle CircleGesture;

        public override void OnInit(Leap.Controller controller)
        {
            if (Initialized != null)
            {
                Initialized("Leap Motion Initialized.");
            }
        }

        public override void OnConnect(Leap.Controller controller)
        {
            controller.EnableGesture(Leap.Gesture.GestureType.TYPECIRCLE);
            controller.EnableGesture(Leap.Gesture.GestureType.TYPEKEYTAP);
            controller.EnableGesture(Leap.Gesture.GestureType.TYPESCREENTAP);
            controller.EnableGesture(Leap.Gesture.GestureType.TYPESWIPE);

            if (Connected != null)
            {
                Connected("Leap Motion Connected !");
            }
        }

        public override void OnDisconnect(Leap.Controller controller)
        {
            if (Disconnected != null)
            {
                Disconnected("Leap Motion Disconnected !");
            }
        }

        public override void OnExit(Leap.Controller controller)
        {
            if (Exited != null)
            {
                Exited("Leap Motion Exited.");
            }
        }

        public override void OnFrame(Leap.Controller controller)
        {
            Leap.Frame frame = controller.Frame();
            Leap.Hand secondHand = null;

            if (!frame.Hands.Empty)
            {
                // Get the first hand
                Leap.Hand hand = frame.Hands[0];

                if (ReturnHand != null && hand.Fingers.Count == 1)
                {
                    ReturnHand(hand.PalmNormal.x, hand.PalmNormal.y, hand.PalmNormal.z, hand.PalmPosition.x, hand.PalmPosition.y, hand.PalmPosition.z, hand.Direction.x, hand.Direction.y, hand.Direction.z);
                }
            }

            if (frame.Hands.Count == 2)
            {
                System.Collections.Generic.List<int> handsId = new System.Collections.Generic.List<int>();

                foreach (Leap.Hand h in frame.Hands)
                {
                    handsId.Add(h.Id);
                }

                secondHand = frame.Hand(handsId[1]);
            }

            if (secondHand != null)
            {
                // Leap.GestureList gestures = frame.Gestures();
                Leap.GestureList gestures = secondHand.Frame.Gestures();

                for (int i = 0; i < gestures.Count; i++)
                {
                    Leap.Gesture gesture = gestures[i];

                    switch (gesture.Type)
                    {
                        case Leap.Gesture.GestureType.TYPECIRCLE:
                            Leap.CircleGesture circle = new Leap.CircleGesture(gesture);

                            // Calculate clock direction using the angle between circle normal and pointable
                            if (circle.Pointable.Direction.AngleTo(circle.Normal) <= Math.PI / 4)
                            {
                                //Clockwise if angle is less than 90 degrees
                                if (CircleGesture != null)
                                {
                                    CircleGesture(circle.Id, "clockwise");
                                }
                            }
                            else
                            {
                                if (CircleGesture != null)
                                {
                                    CircleGesture(circle.Id, "counterclockwise");
                                }
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
}
