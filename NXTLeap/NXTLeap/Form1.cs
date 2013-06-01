using System;

namespace NXTLeap
{
    public partial class NXTLeapForm : System.Windows.Forms.Form
    {
        private static NXTRemoteLib.Sockets.SocketWrapper aSocket = new NXTRemoteLib.Sockets.SocketWrapper();

        private Objects.NXTLeapListener leapListener = new Objects.NXTLeapListener();
        private Leap.Controller leapController = new Leap.Controller();

        private static int lastCircleId = 0;

        private static int EFFORT = 72;
        private const int EFFORT_MULTIPLICATOR = 1;
        private const int EFFORT_UPDATING_VALUE = 10;

        private static order lastStrOrder = order.STOP;

        public NXTLeapForm()
        {
            InitializeComponent();
        }

        private enum order
        {
            FORWARD,
            BACKWARD,
            LEFT,
            RIGHT,
            STOP
        }

        private void btn_Connection_Click(object sender, EventArgs e)
        {
            if (aSocket.IsConnected() == false)
            {
                System.Net.IPAddress ip = System.Net.IPAddress.Parse("127.0.0.1");
                if (System.Net.IPAddress.TryParse(tb_Ip.Text, out ip))
                {
                    aSocket = new NXTRemoteLib.Sockets.SocketWrapper();
                    aSocket.Connected += OnSocketConnected;
                    aSocket.Shutdown += OnSocketClosed;
                    aSocket.ServerClosedConnection += OnSocketClosed;

                    aSocket.ConnectToServer(tb_Ip.Text, Int32.Parse(tb_Port.Text));

                    leapListener = new Objects.NXTLeapListener();
                    leapController = new Leap.Controller();

                    leapListener.Initialized += OnLeapLog;
                    leapListener.Connected += OnLeapLog;
                    leapListener.Disconnected += OnLeapLog;
                    leapListener.Exited += OnLeapLog;
                    leapListener.Framed += OnLeapLog;
                    leapListener.CircleGesture += OnLeapCircleGesture;
                    leapListener.ReturnHand += OnLeapHand;

                    leapController.AddListener(leapListener);
                }
                else
                {
                    System.Windows.Forms.MessageBox.Show("Adresse ip incorrect.");
                }
            }
            else
            {
                aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", "0", "0", "false"));
                aSocket.ShutdownClient();
            }
        }

        private void OnLeapCircleGesture(int id, string direction)
        {
            string log = null;

            if (lastCircleId != id)
            {
                if (direction == "clockwise")
                {
                    EFFORT += EFFORT_UPDATING_VALUE;
                    log = "SPEED INCREASED (" + EFFORT.ToString() + ") !";
                }
                else
                {
                    EFFORT -= EFFORT_UPDATING_VALUE;
                    log = "SPEED DECREASED (" + EFFORT.ToString() + ") !";
                }

                if (lastStrOrder == order.FORWARD && direction == "clockwise")
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", EFFORT.ToString(), "1", "true"));
                }
                else if (lastStrOrder == order.FORWARD && direction == "counterclockwise")
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", EFFORT.ToString(), "1", "true"));
                }
                else if (lastStrOrder == order.BACKWARD && direction == "clockwise")
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", EFFORT.ToString(), "1", "false"));
                }
                else if (lastStrOrder == order.BACKWARD && direction == "counterclockwise")
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", EFFORT.ToString(), "1", "false"));
                }
                else if (lastStrOrder == order.RIGHT && direction == "clockwise")
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("1", EFFORT.ToString(), "1", "true"));
                }
                else if (lastStrOrder == order.RIGHT && direction == "counterclockwise")
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("1", EFFORT.ToString(), "1", "true"));
                }
                else if (lastStrOrder == order.LEFT && direction == "clockwise")
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("1", EFFORT.ToString(), "1", "false"));
                }
                else if (lastStrOrder == order.LEFT && direction == "counterclockwise")
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("1", EFFORT.ToString(), "1", "false"));
                }

                if (log != null)
                {
                    UpdateLogListbox(log);
                }

                lastCircleId = id;
            }
        }

        private void OnLeapHand(float pn_x, float pn_y, float pn_z, float pp_x, float pp_y, float pp_z, float pd_x, float pd_y, float pd_z)
        {
            string log = null;

            if (pn_z < -0.4 && lastStrOrder != order.FORWARD)
            {
                aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", EFFORT.ToString(), "1", "true"));
                lastStrOrder = order.FORWARD;
                log = "Forward !";
            }
            else if (pn_z > 0.4 && lastStrOrder != order.BACKWARD)
            {
                aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", EFFORT.ToString(), "1", "false"));
                lastStrOrder = order.BACKWARD;
                log = "Backward !";
            }
            else if (pn_x > 0.3 && lastStrOrder != order.LEFT)
            {
                aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("1", EFFORT.ToString(), "1", "false"));
                lastStrOrder = order.LEFT;
                log = "Left !";
            }
            else if (pn_x < -0.3 && lastStrOrder != order.RIGHT)
            {
                aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("1", EFFORT.ToString(), "1", "true"));
                lastStrOrder = order.RIGHT;
                log = "Right !";
            }
            else if (lastStrOrder != order.STOP)
            {
                if ((pn_z > -0.4 && pn_z < 0.4) && (pn_x < 0.3 && pn_x > -0.3))
                {
                    aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", "0", "0", "false"));
                    lastStrOrder = order.STOP;
                    log = "Stop !";
                }
            }

            if (log != null)
            {
                UpdateLogListbox(log);
            }
        }

        private void OnLeapLog(string log)
        {
            if (NXTLeapForm.ActiveForm != null)
            {
                if (log == "Leap Motion Exited." || log == "Leap Motion Disconnected !")
                {
                    if (aSocket.IsConnected())
                    {
                        aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", "0", "0", "false"));
                    }
                }
                if (log != null)
                {
                    UpdateLogListbox(log);
                }
            }
        }

        private void OnSocketConnected(System.Net.Sockets.SocketError result, object sender)
        {
            if (result == System.Net.Sockets.SocketError.Success)
            {
                UpdateLogListbox("Socket connected to " + tb_Ip.Text + ":" + tb_Port.Text + ".");
            }
            else
            {
                UpdateLogListbox("Socket connection error.");
            }

            this.Invoke((System.Windows.Forms.MethodInvoker)delegate
            {
                btn_Connection.Text = "Disconnect";
                tb_Ip.Enabled = false;
                tb_Port.Enabled = false;
            });
        }

        private void OnSocketClosed(object sender, EventArgs e)
        {
            if (NXTLeapForm.ActiveForm != null)
            {
                this.Invoke((System.Windows.Forms.MethodInvoker)delegate
                {
                    btn_Connection.Text = "Connect";
                    tb_Ip.Enabled = true;
                    tb_Port.Enabled = true;
                });
            }

            UpdateLogListbox("Socket disconnected from " + tb_Ip.Text + ":" + tb_Port.Text + ".");
        }

        private void UpdateLogListbox(string log)
        {
            if (NXTLeapForm.ActiveForm != null)
            {
                this.Invoke((System.Windows.Forms.MethodInvoker)delegate
                {
                    lb_Logs.Items.Add(log);
                    // Auto scroll
                    lb_Logs.SelectedIndex = lb_Logs.Items.Count - 1;
                    lb_Logs.SelectedIndex = -1;
                });
            }
        }

        private void NXTLeapForm_FormClosed(object sender, System.Windows.Forms.FormClosedEventArgs e)
        {
            if (aSocket.IsConnected())
            {
                aSocket.SendData(NXTRemoteLib.Controllers.XML.SendOrder("0", "0", "0", "false"));
                aSocket.ShutdownClient();
            }

            leapController.RemoveListener(leapListener);
            leapController.Dispose();
        }

    }
}
