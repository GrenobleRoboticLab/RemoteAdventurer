namespace NXTLeap
{
    partial class NXTLeapForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.Windows.Forms.GroupBox logGroupBox;
            this.lb_Logs = new System.Windows.Forms.ListBox();
            this.mainGroupBox = new System.Windows.Forms.GroupBox();
            this.connectionGroupBox = new System.Windows.Forms.GroupBox();
            this.btn_Connection = new System.Windows.Forms.Button();
            this.tb_Port = new System.Windows.Forms.TextBox();
            this.l_Port = new System.Windows.Forms.Label();
            this.tb_Ip = new System.Windows.Forms.TextBox();
            this.l_Ip = new System.Windows.Forms.Label();
            logGroupBox = new System.Windows.Forms.GroupBox();
            logGroupBox.SuspendLayout();
            this.mainGroupBox.SuspendLayout();
            this.connectionGroupBox.SuspendLayout();
            this.SuspendLayout();
            // 
            // logGroupBox
            // 
            logGroupBox.Controls.Add(this.lb_Logs);
            logGroupBox.Location = new System.Drawing.Point(6, 107);
            logGroupBox.Name = "logGroupBox";
            logGroupBox.Size = new System.Drawing.Size(357, 279);
            logGroupBox.TabIndex = 1;
            logGroupBox.TabStop = false;
            logGroupBox.Text = "Logs";
            // 
            // lb_Logs
            // 
            this.lb_Logs.FormattingEnabled = true;
            this.lb_Logs.Location = new System.Drawing.Point(6, 19);
            this.lb_Logs.Name = "lb_Logs";
            this.lb_Logs.Size = new System.Drawing.Size(340, 251);
            this.lb_Logs.TabIndex = 0;
            // 
            // mainGroupBox
            // 
            this.mainGroupBox.Controls.Add(logGroupBox);
            this.mainGroupBox.Controls.Add(this.connectionGroupBox);
            this.mainGroupBox.Location = new System.Drawing.Point(12, 12);
            this.mainGroupBox.Name = "mainGroupBox";
            this.mainGroupBox.Size = new System.Drawing.Size(373, 392);
            this.mainGroupBox.TabIndex = 0;
            this.mainGroupBox.TabStop = false;
            // 
            // connectionGroupBox
            // 
            this.connectionGroupBox.Controls.Add(this.btn_Connection);
            this.connectionGroupBox.Controls.Add(this.tb_Port);
            this.connectionGroupBox.Controls.Add(this.l_Port);
            this.connectionGroupBox.Controls.Add(this.tb_Ip);
            this.connectionGroupBox.Controls.Add(this.l_Ip);
            this.connectionGroupBox.Location = new System.Drawing.Point(6, 19);
            this.connectionGroupBox.Name = "connectionGroupBox";
            this.connectionGroupBox.Size = new System.Drawing.Size(357, 82);
            this.connectionGroupBox.TabIndex = 0;
            this.connectionGroupBox.TabStop = false;
            this.connectionGroupBox.Text = "Connection";
            // 
            // btn_Connection
            // 
            this.btn_Connection.Location = new System.Drawing.Point(239, 22);
            this.btn_Connection.Name = "btn_Connection";
            this.btn_Connection.Size = new System.Drawing.Size(107, 46);
            this.btn_Connection.TabIndex = 4;
            this.btn_Connection.Text = "Connect";
            this.btn_Connection.UseVisualStyleBackColor = true;
            this.btn_Connection.Click += new System.EventHandler(this.btn_Connection_Click);
            // 
            // tb_Port
            // 
            this.tb_Port.Location = new System.Drawing.Point(76, 48);
            this.tb_Port.Name = "tb_Port";
            this.tb_Port.Size = new System.Drawing.Size(157, 20);
            this.tb_Port.TabIndex = 3;
            // 
            // l_Port
            // 
            this.l_Port.AutoSize = true;
            this.l_Port.Location = new System.Drawing.Point(6, 51);
            this.l_Port.Name = "l_Port";
            this.l_Port.Size = new System.Drawing.Size(35, 13);
            this.l_Port.TabIndex = 2;
            this.l_Port.Text = "Port : ";
            // 
            // tb_Ip
            // 
            this.tb_Ip.Location = new System.Drawing.Point(76, 22);
            this.tb_Ip.Name = "tb_Ip";
            this.tb_Ip.Size = new System.Drawing.Size(157, 20);
            this.tb_Ip.TabIndex = 1;
            // 
            // l_Ip
            // 
            this.l_Ip.AutoSize = true;
            this.l_Ip.Location = new System.Drawing.Point(6, 25);
            this.l_Ip.Name = "l_Ip";
            this.l_Ip.Size = new System.Drawing.Size(64, 13);
            this.l_Ip.TabIndex = 0;
            this.l_Ip.Text = "IP Address :";
            // 
            // NXTLeapForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(397, 416);
            this.Controls.Add(this.mainGroupBox);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.Fixed3D;
            this.Name = "NXTLeapForm";
            this.Text = "NXTLeap";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.NXTLeapForm_FormClosed);
            logGroupBox.ResumeLayout(false);
            this.mainGroupBox.ResumeLayout(false);
            this.connectionGroupBox.ResumeLayout(false);
            this.connectionGroupBox.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.GroupBox mainGroupBox;
        private System.Windows.Forms.ListBox lb_Logs;
        private System.Windows.Forms.GroupBox connectionGroupBox;
        private System.Windows.Forms.Button btn_Connection;
        private System.Windows.Forms.TextBox tb_Port;
        private System.Windows.Forms.Label l_Port;
        private System.Windows.Forms.TextBox tb_Ip;
        private System.Windows.Forms.Label l_Ip;
    }
}

