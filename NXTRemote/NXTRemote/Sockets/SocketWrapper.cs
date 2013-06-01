using System;

namespace NXTRemote.Sockets
{

    #region Delegates

    // Delegate that will be used to produce the events
    public delegate void DelegateConnected(System.Net.Sockets.SocketError result, object sender);
    public delegate void DelegateSendingDatas(Int32 result, object sender);
    public delegate void DelegateReceivingDatas(byte[] receivedDatas, System.Net.Sockets.SocketError result, object sender);

    #endregion

    public class SocketWrapper : IDisposable
    {

        #region Attributes

        /// <summary>
        /// Private-usage field that determines whether the object has begun listening
        /// </summary>
        private bool _hasStarted = false;
        /// <summary>
        /// Private-usage field that determines the buffer length
        /// </summary>
        private int _maxBufferLength = 2048;
        /// <summary>
        /// The main Socket
        /// </summary>
        protected System.Net.Sockets.Socket _mainSocket = null;
        /// <summary>
        /// Byte array buffer that will contain recieved data
        /// </summary>
        protected byte[] _buffer = null;
        /// <summary>
        /// Private-usage field that holds last recieved data
        /// </summary>
        protected byte[] _receivedDatas = null;
        /// <summary>
        /// The DnsEndPoint that holds the information of the host
        /// </summary>
        protected System.Net.IPEndPoint _ipEndPoint = null;
        /// <summary>
        /// Object that is passed onto Socket.ReceiveAsync to collect data in its buffer
        /// </summary>
        protected System.Net.Sockets.SocketAsyncEventArgs _receiveSocketArgs = null;
        /// <summary>
        /// Object that is passed onto Socket.SendAsync to determine completion
        /// </summary>
        protected System.Net.Sockets.SocketAsyncEventArgs _sendSocketArgs = null;

        #endregion

        #region Properties

        /// <summary>
        /// Property that contains the last stored received data as byte array
        /// </summary>
        public byte[] LastReceivedDatas
        {
            get { return _receivedDatas; }
        }

        /// <summary>
        /// Property to access the default Encoding to be used in the asynchronous operations
        /// </summary>
        public System.Text.Encoding DefaultEncoding
        {
            get;
            set;
        }

        /// <summary>
        /// Property that contains the maximum-length buffer size to be used once the connection has been established.
        /// </summary>
        public int BufferLength
        {
            get
            {
                return _maxBufferLength;
            }
            set
            {
                _maxBufferLength = value;
            }
        }

        #endregion

        #region Enums

        private enum CompletedType
        {
            /// <summary>
            /// Private-usage enumeration sent as a usertoken to determine 
            /// completion of an asynchronous connection
            /// </summary>
            Connected_Complete,
            /// <summary>
            /// Private-usage enumeration sent as a usertoken to determine 
            /// completion of an asynchronous data retrieval
            /// </summary>
            Receiving_Complete,
            /// <summary>
            /// Private-usage enumeration sent as a usertoken to determine 
            /// completion of an asynchronous data-distribution 
            /// </summary>
            Sending_Complete
        };

        #endregion

        #region Events

        // Events that outside code can subscribe to
        /// <summary>
        /// An event a client can subscribe to determine connection
        /// </summary>
        public event DelegateConnected Connected;
        /// <summary>
        /// An event a client can subscribe to determine data retrieval
        /// </summary>
        public event DelegateReceivingDatas Receiving;
        /// <summary>
        /// An event a client can subscribe to determine data-distribution completion
        /// </summary>
        public event DelegateSendingDatas Sending;
        /// <summary>
        /// An event a client can subscribe to determine Shutdown completion
        /// </summary>
        public event EventHandler Shutdown;
        /// <summary>
        /// An event a client can subscribe to determine when host has terminated the connection
        /// </summary>
        public event EventHandler ServerClosedConnection;

        #endregion

        #region Methods

        /// <summary>
        /// Initializes internal fields
        /// </summary>
        private void InitializeData()
        {
            // Initialize the Socket
            _mainSocket = new System.Net.Sockets.Socket(System.Net.Sockets.AddressFamily.InterNetwork, System.Net.Sockets.SocketType.Stream, System.Net.Sockets.ProtocolType.Tcp);
            DefaultEncoding = System.Text.Encoding.UTF8;
            _receiveSocketArgs = new System.Net.Sockets.SocketAsyncEventArgs();
            _sendSocketArgs = new System.Net.Sockets.SocketAsyncEventArgs();
        }

        /// <summary>
        /// Private-usage method to manage the recieved data from the socket
        /// </summary>
        private void OnCompleted(object sender, System.Net.Sockets.SocketAsyncEventArgs args)
        {
            switch ((CompletedType)args.UserToken)
            {
                case CompletedType.Connected_Complete:
                    OnConnect(sender, args);
                    break;
                case CompletedType.Receiving_Complete:
                    OnReceive(sender, args);
                    break;
                case CompletedType.Sending_Complete:
                    OnSent(sender, args);
                    break;
                default:
                    throw new Exception("Unknown User Token Provided.");
            }
        }

        /// <summary>
        /// Connect to a listening server
        /// </summary>
        /// <remarks>The method will throw an exception if the Client is already connected, already listening, 
        /// if the provided port number is out of Silverlight's range, or if the provided host name string is unusable (empty or null).</remarks>
        /// <param name="host">String specifying either the hostname or the ip address</param>
        /// <param name="portNumber">The port number in which the object will attempt to connect to</param>
        /// <returns>Returns 1 if the operation is pending as an asynchronous process. Returns 0 if the operation will be performed synchronously.</returns>
        public int ConnectToServer(string host, Int32 portNumber)
        {
            if (_hasStarted)
            {
                throw new Exception("Client either already listening !");
            }
            else
            {
                InitializeData();

                if (IsConnected())
                {
                    throw new Exception("Client already connected !");
                }

                if (host == null || host == "")
                {
                    throw new Exception("Provided host string is empty or null !");
                }

                // Fill the structure that will represent the server
                _ipEndPoint = new System.Net.IPEndPoint(System.Net.IPAddress.Parse(host), portNumber);

                // Create an object that will serve as the medium for our connection transaction
                _receiveSocketArgs.RemoteEndPoint = _ipEndPoint;

                // Instantiate the SocketAsyncArgs that will be used to receive the data
                _receiveSocketArgs.Completed += new EventHandler<System.Net.Sockets.SocketAsyncEventArgs>(OnCompleted);
                _receiveSocketArgs.UserToken = CompletedType.Connected_Complete;

                // Instantiate the SocketAsyncArgs that will be used to send the data
                _sendSocketArgs.Completed += new EventHandler<System.Net.Sockets.SocketAsyncEventArgs>(OnCompleted);
                _sendSocketArgs.UserToken = CompletedType.Sending_Complete;

                // Connect to Server Asynchronously 
                if (!_mainSocket.ConnectAsync(_receiveSocketArgs))
                {
                    // The task will be performed synchronously 
                    return 1;
                }
            }

            // The task will be performed asynchronously
            return 0;
        }

        /// <summary>
        /// Private-usage method to manage all asynchronous invocations
        /// </summary>
        private void OnConnect(object sender, System.Net.Sockets.SocketAsyncEventArgs args)
        {
            // Was the connection successful or unsuccessful
            if (args.SocketError == System.Net.Sockets.SocketError.Success)
            {
                _hasStarted = true;

                // Set the buffer in which we will use to directly write/read to the stream
                _buffer = new byte[_maxBufferLength];
                _receiveSocketArgs.SetBuffer(_buffer, 0, _buffer.Length);

                // Modify the user token
                args.UserToken = CompletedType.Receiving_Complete;

                // Start Listening
                _mainSocket.ReceiveAsync(_receiveSocketArgs);
            }

            // Notify any subscribers
            if (Connected != null)
            {
                Connected(args.SocketError, this);
            }
        }

        /// <summary>
        /// Private-usage method to manage data retrieval
        /// </summary>
        private void OnReceive(object sender, System.Net.Sockets.SocketAsyncEventArgs e)
        {
            byte[] obtainedData = new byte[_maxBufferLength];

            try
            {
                using (System.IO.MemoryStream memStream = new System.IO.MemoryStream(e.Buffer))
                {
                    memStream.Read(obtainedData, e.Offset, e.BytesTransferred);
                }

            }
            catch (Exception ex)
            {
                throw new Exception("Socket Client: Could not read from stream.\n" + ex.Message);
            }

            if (e.SocketError != System.Net.Sockets.SocketError.Success)
            {
                // Host has abruptly disconnected
                if (e.SocketError == System.Net.Sockets.SocketError.ConnectionReset)
                {
                    OnDisconnect();
                    return;
                }

                // Notify the subscribers of the error
                if (Receiving != null)
                {
                    Receiving(null, e.SocketError, this);
                }
            }
            else
            {
                // Store the data
                _receivedDatas = obtainedData;

                // Notify the subscribers of a successful operation
                if (Receiving != null)
                {
                    Receiving(obtainedData, e.SocketError, this);
                }

                // Begin to listen yet again
                _mainSocket.ReceiveAsync(_receiveSocketArgs);
            }
        }

        private void OnDisconnect()
        {
            // Allow the user to reconnect
            _hasStarted = false;

            // Remote host closed connection
            if (ServerClosedConnection != null)
            {
                ServerClosedConnection(this, null);
            }
        }

        /// <summary>
        /// Sends data asynchronously, with float-type data
        /// </summary>
        public bool SendData(float datas)
        {
            return SendData(BitConverter.GetBytes(datas));
        }

        /// <summary>
        /// Sends data asynchronously, with long-type data
        /// </summary>
        public bool SendData(long datas)
        {
            return SendData(BitConverter.GetBytes(datas));
        }

        /// <summary>
        /// Sends data asynchronously, with boolean data
        /// </summary>
        public bool SendData(bool datas)
        {
            return SendData(BitConverter.GetBytes(datas));
        }

        /// <summary>
        /// Sends data asynchronously, with double-type data
        /// </summary>
        public bool SendData(double datas)
        {
            return SendData(BitConverter.GetBytes(datas));
        }

        /// <summary>
        /// Sends data asynchronously, with integer-type data
        /// </summary>
        public bool SendData(int datas)
        {
            return SendData(BitConverter.GetBytes(datas));
        }

        /// <summary>
        /// Sends data asynchronously, with string and encoding
        /// </summary>
        public bool SendData(string datas, string encoding)
        {
            try
            {
                return SendData(System.Text.Encoding.GetEncoding(encoding).GetBytes(datas));
            }
            catch
            {
                throw new Exception("Socket Client: Encoding provided not recognized !");
            }
        }

        /// <summary>
        /// Sends data asynchronously, with string and UTF-8
        /// </summary>
        public bool SendData(string datas)
        {
            return SendData(DefaultEncoding.GetBytes(datas + "\r\n"));
        }

        /// <summary>
        /// Sends data asynchronously, with bytes
        /// </summary>
        public bool SendData(byte[] datas)
        {
            if (!IsConnected() && _hasStarted == false)
            {
                throw new Exception("Client is not connected !");
            }

            _sendSocketArgs.RemoteEndPoint = _ipEndPoint;
            _sendSocketArgs.SetBuffer(datas, 0, datas.Length);

            try
            {
                return _mainSocket.SendAsync(_sendSocketArgs);
            }
            catch (Exception ex)
            {
                throw new Exception("Socket Client : Could not send data.\n" + ex.Message);
            }
        }

        /// <summary>
        /// Private-usage method to manage data sending completion
        /// </summary>
        private void OnSent(object sender, System.Net.Sockets.SocketAsyncEventArgs e)
        {
            // This should probably be overriden later on, by inherited classes
            if (Sending != null)
            {
                Sending((e.SocketError == System.Net.Sockets.SocketError.Success) ? 0 : 1, this);
            }
        }

        /// <summary>
        /// Returns a string representation of the last data recieved
        /// </summary>
        public string GetRecievedDataAsString()
        {
            return DefaultEncoding.GetString(LastReceivedDatas, 0, LastReceivedDatas.Length);
        }

        /// <summary>
        /// Returns a string representation of the last data recieved
        /// </summary>
        /// <param name="encoding">Optional Encoding to parse the information</param>
        public string GetRecievedDataAsString(string encoding)
        {
            return System.Text.Encoding.GetEncoding(encoding).GetString(LastReceivedDatas, 0, LastReceivedDatas.Length);
        }

        /// <summary>
        /// Accessor Method for the Socket
        /// </summary>
        /// <remarks>Be weary of manipulating the Socket using this interface.</remarks>
        /// <returns>A System.Net.Socket.Socket object is returned, representing the current connection.</returns>
        public System.Net.Sockets.Socket GetSocket()
        {
            return _mainSocket;
        }

        /// <summary>
        /// Closes the connection and disposes unmanaged resources
        /// </summary>
        /// <remarks>The method shuts down the server by disposing of the Socket and the SocketAsyncEvent args utilized in the transaction processes.</remarks>
        public void ShutdownClient()
        {
            // TODO: Fix this
            Dispose();
        }

        /// <summary>
        /// Returns true if the Socket is sconnected to a host
        /// </summary>
        public bool IsConnected()
        {
            if (_mainSocket != null)
            {
                return _mainSocket.Connected;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Constructor for initializing private fields.
        /// </summary>
        public SocketWrapper()
        {
            InitializeData();
        }

        /// <summary>
        /// Destructor
        /// </summary>
        ~SocketWrapper()
        {
            ShutdownClient();
        }

        #endregion

        #region IDisposable Members

        /// <summary>
        /// Disposes all the unmanaged resources in the object. It is equivalent to calling Shutdown()
        /// </summary>
        public void Dispose()
        {
            if (_mainSocket != null)
            {
                if (IsConnected())
                {
                    _mainSocket.Close();
                }
            }

            if (_receiveSocketArgs != null)
            {
                _receiveSocketArgs.Dispose();
            }

            _hasStarted = false;

            if (Shutdown != null)
            {
                Shutdown(this, null);
            }
        }

        #endregion

    }
}
