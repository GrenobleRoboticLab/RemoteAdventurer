using System;

namespace NXTRemote.Sockets
{
    public class TwoSensesSocket
    {

        #region Attributes

        // Catch all incoming asynchronous socket operation.
        private System.Net.Sockets.SocketAsyncEventArgs p_socketReceiveEventArgs = new System.Net.Sockets.SocketAsyncEventArgs();
        // Catch all outgoing asynchronous socket operation.
        private System.Net.Sockets.SocketAsyncEventArgs p_socketSendEventArgs = new System.Net.Sockets.SocketAsyncEventArgs();
        // Implements the Berkeley sockets interface.
        private System.Net.Sockets.Socket p_twoSensesSocket = new System.Net.Sockets.Socket(System.Net.Sockets.AddressFamily.InterNetwork, System.Net.Sockets.SocketType.Stream, System.Net.Sockets.ProtocolType.Tcp);

        System.Net.IPEndPoint endPoint = null;

        // The size of the buffer for receiving datas.
        private const int BufferSize = 2048;

        private static System.Threading.ManualResetEvent receiveDone = new System.Threading.ManualResetEvent(false);

        private static object token = new object();

        // Delegate that manage events.
        public delegate void TwoSensesSocketEventHandler(object sender, EventArgs e);
        public delegate void TwoSensesSocketEventHandlerSend(object sender, EventArgs e);

        // Event fired when a socket is connected.
        public event TwoSensesSocketEventHandler Connected;
        // Event fired when a socket is disconnected.
        public event TwoSensesSocketEventHandler Disconnected;
        // Event fired while getting datas.
        public event TwoSensesSocketEventHandler IncomingDatas;
        // Event fired while sending datas.
        public event TwoSensesSocketEventHandler OutgoingDatas;
        // Event fired while a socket error occurs.
        public event TwoSensesSocketEventHandler SocketError;

        #endregion


        #region Constructors

        /// <summary>
        /// Default constructor.
        /// </summary>
        public TwoSensesSocket()
        {
            // Subscribe for receiving data.
            p_socketReceiveEventArgs.Completed += new EventHandler<System.Net.Sockets.SocketAsyncEventArgs>(SocketReceiveEventArgs_Completed);
            // Subscribe for sending data.
            // p_socketSendEventArgs.Completed += new EventHandler<System.Net.Sockets.SocketAsyncEventArgs>(SocketSendEventArgs_Completed);
        }

        /// <summary>
        /// Constructor to specify the socket parameters.
        /// </summary>
        /// <param name="socketAddressFamily"></param>
        /// <param name="socketType"></param>
        /// <param name="socketProtocolType"></param>
        public TwoSensesSocket(System.Net.Sockets.AddressFamily socketAddressFamily, System.Net.Sockets.SocketType socketType, System.Net.Sockets.ProtocolType socketProtocolType)
        {
            // Subscribe for receiving data.
            p_socketReceiveEventArgs.Completed += new EventHandler<System.Net.Sockets.SocketAsyncEventArgs>(SocketReceiveEventArgs_Completed);
            // Subscribe for sending data.
            // p_socketSendEventArgs.Completed += new EventHandler<System.Net.Sockets.SocketAsyncEventArgs>(SocketSendEventArgs_Completed);
            // Initialize the socket.
            p_twoSensesSocket = new System.Net.Sockets.Socket(socketAddressFamily, socketType, socketProtocolType);
        }

        #endregion


        #region Callbacks

        /// <summary>
        /// Callback when datas are received.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void SocketReceiveEventArgs_Completed(object sender, System.Net.Sockets.SocketAsyncEventArgs e)
        {
            string datas = null;

            switch (e.LastOperation)
            {
                case System.Net.Sockets.SocketAsyncOperation.Connect:
                    if (e.SocketError == System.Net.Sockets.SocketError.Success)
                    {
                        receiveDone.Set();
                        // Prepare for receiving datas.
                        byte[] buffer = new byte[BufferSize];
                        e.SetBuffer(buffer, 0, buffer.Length);
                        receiveDone.Reset();
                        p_twoSensesSocket.ReceiveAsync(p_socketReceiveEventArgs);
                        if (Connected != null)
                        {
                            // Return the System.Net.Sockets.SocketError object.
                            Connected(e.SocketError, EventArgs.Empty);
                        }
                    }
                    else
                    {
                        if (SocketError != null)
                        {
                            // Return the System.Net.Sockets.SocketError object.
                            SocketError(e.SocketError, EventArgs.Empty);
                        }
                    }
                    break;
                case System.Net.Sockets.SocketAsyncOperation.Receive:
                    if (e.SocketError == System.Net.Sockets.SocketError.Success)
                    {
                        receiveDone.Set();
                        datas = System.Text.Encoding.UTF8.GetString(e.Buffer, 0, e.BytesTransferred);
                        receiveDone.Reset();
                        p_twoSensesSocket.ReceiveAsync(p_socketReceiveEventArgs);
                        if (IncomingDatas != null)
                        {
                            IncomingDatas(datas, EventArgs.Empty);
                        }
                    }
                    else
                    {
                        if (SocketError != null)
                        {
                            // Return the System.Net.Sockets.SocketError object.
                            SocketError(e.SocketError, EventArgs.Empty);
                        }
                    }
                    break;
                case System.Net.Sockets.SocketAsyncOperation.Send:
                    if (e.SocketError == System.Net.Sockets.SocketError.Success)
                    {
                        receiveDone.Set();
                        datas = System.Text.Encoding.UTF8.GetString(e.Buffer, 0, e.BytesTransferred);
                        p_socketReceiveEventArgs.SetBuffer(e.Buffer, 0, e.Buffer.Length);
                        p_twoSensesSocket.SendAsync(p_socketReceiveEventArgs);
                        receiveDone.Reset();
                        if (OutgoingDatas != null)
                        {
                            OutgoingDatas(datas, EventArgs.Empty);
                        }
                    }
                    else
                    {
                        if (SocketError != null)
                        {
                            // Return the System.Net.Sockets.SocketError object.
                            SocketError(e.SocketError, EventArgs.Empty);
                        }
                    }


                    if (OutgoingDatas != null)
                    {
                        // Return the System.Net.Sockets.SocketError object.
                        OutgoingDatas(e.SocketError, EventArgs.Empty);
                    }
                    break;
            }
        }

        /// <summary>
        /// Callback when datas are sended.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void SocketSendEventArgs_Completed(object sender, System.Net.Sockets.SocketAsyncEventArgs e)
        {
            if (e.LastOperation == System.Net.Sockets.SocketAsyncOperation.Connect)
            {
                receiveDone.Set();
                // Prepare for receiving datas.
                byte[] buffer = new byte[BufferSize];
                e.SetBuffer(buffer, 0, buffer.Length);
                receiveDone.Reset();
                p_twoSensesSocket.ReceiveAsync(p_socketReceiveEventArgs);
            }
            if (e.LastOperation == System.Net.Sockets.SocketAsyncOperation.Send)
            {
                if (OutgoingDatas != null)
                {
                    // Return the System.Net.Sockets.SocketError object.
                    OutgoingDatas(e.SocketError, EventArgs.Empty);
                }
            }
            else if (e.LastOperation == System.Net.Sockets.SocketAsyncOperation.Receive)
            {
            }

        }

        #endregion


        #region Methods

        /// <summary>
        /// Connect the socket.
        /// </summary>
        /// <param name="ipAddress">Ip address of the server.</param>
        /// <param name="portNumber">port of the server.</param>
        public void Connect(string ipAddress, string portNumber)
        {
            System.Net.IPAddress ip = System.Net.IPAddress.Parse(ipAddress);
            endPoint = new System.Net.IPEndPoint(ip, Int32.Parse(portNumber));

            p_socketReceiveEventArgs.RemoteEndPoint = endPoint;
            // p_socketSendEventArgs.RemoteEndPoint = endPoint;

            p_twoSensesSocket = new System.Net.Sockets.Socket(System.Net.Sockets.AddressFamily.InterNetwork, System.Net.Sockets.SocketType.Stream, System.Net.Sockets.ProtocolType.Tcp);
            p_twoSensesSocket.ConnectAsync(p_socketReceiveEventArgs);

            // p_twoSensesSocket.ConnectAsync(p_socketSendEventArgs);

            receiveDone.WaitOne();
        }

        /// <summary>
        /// Send datas.
        /// </summary>
        /// <param name="data">Datas to send.</param>
        public void Send(string data)
        {
            System.Net.Sockets.SocketAsyncEventArgs args = new System.Net.Sockets.SocketAsyncEventArgs();

            if (p_twoSensesSocket.Connected)
            {
                receiveDone.Set();
                byte[] buffer = System.Text.Encoding.UTF8.GetBytes(data + String.Empty);
                // p_socketSendEventArgs.SetBuffer(buffer, 0, buffer.Length);
                args.Completed += new EventHandler<System.Net.Sockets.SocketAsyncEventArgs>(SocketSendEventArgs_Completed);
                args.SetBuffer(buffer, 0, buffer.Length);
                args.RemoteEndPoint = endPoint;
                p_twoSensesSocket.ConnectAsync(args);
                receiveDone.Reset();
            }
        }

        /// <summary>
        /// Return the socket state.
        /// </summary>
        /// <returns>True if the socket is connected, or false.</returns>
        public bool IsConnected()
        {
            return p_twoSensesSocket.Connected;
        }

        /// <summary>
        /// Disconnect the socket.
        /// </summary>
        public void Disconnect()
        {
            if (p_twoSensesSocket.Connected)
            {
                p_twoSensesSocket.Close();
                if (Disconnected != null)
                {
                    Disconnected(true, EventArgs.Empty);
                }
            }
        }

        #endregion

    }
}