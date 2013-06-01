using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using Microsoft.Phone.Controls;

namespace NXTRemote
{
    public partial class MainPage : PhoneApplicationPage
    {

        private static NXTRemote.Sockets.SocketWrapper aSocket = new NXTRemote.Sockets.SocketWrapper();
        private static string[] lastOrder = new string[4] { null, null, null, null };
        private const int EFFORT_MULTIPLICATOR = 3;

        // Constructor
        public MainPage()
        {
            InitializeComponent();
        }

        private void ButtonConnection_Click(object sender, RoutedEventArgs e)
        {
            // rightMotorGaugeNeedle.Value = Convert.ToDouble((Int32)Math.Ceiling(Math.Abs(updateEntity.LeftWheel.Effort / EFFORT_MULTIPLICATOR)));

            if (aSocket.IsConnected() == false)
            {
                aSocket.Connected += OnConnect;
                aSocket.Shutdown += OnDisconnect;
                aSocket.Receiving += OnIncomingDatas;
                aSocket.ConnectToServer(TextboxIp.Text, Int32.Parse(TextboxPort.Text));
            }
            else
            {
                aSocket.ShutdownClient();
            }
        }

        // de 0 à 300
        private void ButtonStop_Click(object sender, RoutedEventArgs e)
        {
            string order = NXTRemote.Controllers.XML.SendOrder("0", "0", "0", "true");
            lastOrder[0] = "0";
            lastOrder[1] = "0";
            lastOrder[2] = "0";
            lastOrder[3] = "true";
            aSocket.SendData(order);
        }

        private void ButtonForward_Click(object sender, RoutedEventArgs e)
        {
            string order = NXTRemote.Controllers.XML.SendOrder("0", speedSlider.Value.ToString(), EFFORT_MULTIPLICATOR.ToString(), "true");
            lastOrder[0] = "0";
            lastOrder[1] = speedSlider.Value.ToString();
            lastOrder[2] = EFFORT_MULTIPLICATOR.ToString();
            lastOrder[3] = "true";
            aSocket.SendData(order);
        }

        private void ButtonBack_Click(object sender, RoutedEventArgs e)
        {
            string order = NXTRemote.Controllers.XML.SendOrder("0", speedSlider.Value.ToString(), EFFORT_MULTIPLICATOR.ToString(), "false");
            lastOrder[0] = "0";
            lastOrder[1] = speedSlider.Value.ToString();
            lastOrder[2] = EFFORT_MULTIPLICATOR.ToString();
            lastOrder[3] = "false";
            aSocket.SendData(order);
        }

        private void ButtonLeft_Click(object sender, RoutedEventArgs e)
        {
            string order = NXTRemote.Controllers.XML.SendOrder("1", speedSlider.Value.ToString(), EFFORT_MULTIPLICATOR.ToString(), "false");
            lastOrder[0] = "1";
            lastOrder[1] = speedSlider.Value.ToString();
            lastOrder[2] = EFFORT_MULTIPLICATOR.ToString();
            lastOrder[3] = "false";
            aSocket.SendData(order);
        }

        private void ButtonRight_Click(object sender, RoutedEventArgs e)
        {
            string order = NXTRemote.Controllers.XML.SendOrder("1", speedSlider.Value.ToString(), EFFORT_MULTIPLICATOR.ToString(), "true");
            lastOrder[0] = "1";
            lastOrder[1] = speedSlider.Value.ToString();
            lastOrder[2] = EFFORT_MULTIPLICATOR.ToString();
            lastOrder[3] = "true";
            aSocket.SendData(order);
        }

        private void OnIncomingDatas(byte[] receivedDatas, System.Net.Sockets.SocketError result, object sender)
        {
            string dashboard = Controllers.XML.DashboardToString(receivedDatas);

            if (dashboard != null)
            {
                if (dashboard.StartsWith("<?xml"))
                {
                    // BeginInvoke avoid a crash while updating the UI
                    Deployment.Current.Dispatcher.BeginInvoke(() =>
                    {
                        Objects.NXTEntity updateEntity = Controllers.XML.DashboardToNXTEntity(dashboard);

                        leftContactCheckbox.IsChecked = updateEntity.LeftContact.IsTouching;
                        leftRangeSlider.Minimum = updateEntity.Ultrasonic.RangeMin;
                        leftRangeSlider.Maximum = updateEntity.Ultrasonic.RangeMax;
                        leftRangeSlider.Value = updateEntity.Ultrasonic.Range;

                        if (updateEntity.Ultrasonic.Range.ToString().Length == 1)
                        {
                            rangeTextBlock.Text = "   " + updateEntity.Ultrasonic.Range.ToString() + " cm";
                        }
                        else if (updateEntity.Ultrasonic.Range.ToString().Length == 2)
                        {
                            rangeTextBlock.Text = "  " + updateEntity.Ultrasonic.Range.ToString() + " cm";
                        }
                        else if (updateEntity.Ultrasonic.Range.ToString().Length == 3)
                        {
                            rangeTextBlock.Text = " " + updateEntity.Ultrasonic.Range.ToString() + " cm";
                        }
                        else
                        {
                            rangeTextBlock.Text = updateEntity.Ultrasonic.Range.ToString() + " cm";
                        }

                        rightRangeSlider.Minimum = updateEntity.Ultrasonic.RangeMin;
                        rightRangeSlider.Maximum = updateEntity.Ultrasonic.RangeMax;
                        rightRangeSlider.Value = updateEntity.Ultrasonic.Range;
                        rightContactCheckbox.IsChecked = updateEntity.RightContact.IsTouching;

                        Color resetColor = new Color();
                        resetColor.A = 255;
                        resetColor.B = Byte.Parse(updateEntity.Color.Blue.ToString());
                        resetColor.G = Byte.Parse(updateEntity.Color.Green.ToString());
                        resetColor.R = Byte.Parse(updateEntity.Color.Red.ToString());
                        colorRectangle.Fill = new SolidColorBrush(resetColor);


                        if (updateEntity.LeftWheel.Effort < 0)
                        {
                            updateEntity.LeftWheel.Effort = Math.Abs(updateEntity.LeftWheel.Effort);
                        }

                        if (updateEntity.RightWheel.Effort < 0)
                        {
                            updateEntity.RightWheel.Effort = Math.Abs(updateEntity.RightWheel.Effort);
                        }

                        if (updateEntity.AuxWheel.Effort < 0)
                        {
                            updateEntity.AuxWheel.Effort = Math.Abs(updateEntity.AuxWheel.Effort);
                        }

                        leftMotorGaugeNeedle.Value = Convert.ToDouble((Int32)Math.Ceiling(updateEntity.LeftWheel.Effort / EFFORT_MULTIPLICATOR));
                        rightMotorGaugeNeedle.Value = Convert.ToDouble((Int32)Math.Ceiling(updateEntity.RightWheel.Effort / EFFORT_MULTIPLICATOR));
                        auxMotorGaugeNeedle.Value = Convert.ToDouble((Int32)Math.Ceiling(updateEntity.AuxWheel.Effort / EFFORT_MULTIPLICATOR));

                    });
                }
            }
        }

        private void OnConnect(System.Net.Sockets.SocketError result, object sender)
        {
            Deployment.Current.Dispatcher.BeginInvoke(() =>
            {
                ButtonConnection.Content = "Disconnect";
                TextboxIp.IsEnabled = false;
                TextboxPort.IsEnabled = false;

                ButtonForward.IsEnabled = true;
                ButtonRight.IsEnabled = true;
                ButtonStop.IsEnabled = true;
                ButtonLeft.IsEnabled = true;
                ButtonBack.IsEnabled = true;

                speedSlider.IsHitTestVisible = true;
            });
        }

        private void OnDisconnect(object sender, EventArgs e)
        {
            Deployment.Current.Dispatcher.BeginInvoke(() =>
            {
                ButtonConnection.Content = "Connect";
                TextboxIp.IsEnabled = true;
                TextboxPort.IsEnabled = true;

                leftContactCheckbox.IsChecked = false;
                leftRangeSlider.Value = 0;
                rangeTextBlock.Text = " 0 cm";
                rightRangeSlider.Value = 0;
                rightContactCheckbox.IsChecked = false;

                Color resetColor = new Color();
                resetColor.A = 0;
                resetColor.B = 0;
                resetColor.G = 0;
                resetColor.R = 0;
                colorRectangle.Fill = new SolidColorBrush(resetColor);

                leftMotorGaugeNeedle.Value = 0;
                rightMotorGaugeNeedle.Value = 0;
                auxMotorGaugeNeedle.Value = 0;

                ButtonForward.IsEnabled = false;
                ButtonRight.IsEnabled = false;
                ButtonStop.IsEnabled = false;
                ButtonLeft.IsEnabled = false;
                ButtonBack.IsEnabled = false;

                speedSlider.IsHitTestVisible = false;
            });
        }

        private void speedSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            int speedPerc = 0;

            if (lastOrder[0] != null)
            {
                string order = NXTRemote.Controllers.XML.SendOrder(lastOrder[0], speedSlider.Value.ToString(), lastOrder[2], lastOrder[3]);
                aSocket.SendData(order);
            }

            if (speedPercentage != null)
            {
                speedPerc = Convert.ToInt32(Math.Ceiling(Math.Abs(speedSlider.Value)).ToString());

                if (speedPerc.ToString().Length == 1)
                {
                    speedPercentage.Text = "  " + speedPerc + " %";
                }
                else if (speedPerc.ToString().Length == 2)
                {
                    speedPercentage.Text = " " + speedPerc + " %";
                }
                else
                {
                    speedPercentage.Text = speedPerc + " %";
                }
            }
        }

    }
}