using System;

namespace NXTRemote.Controllers
{
    public class XML
    {

        public static System.Xml.XmlReader DashboardToXml(string Dashboard)
        {
            return System.Xml.XmlReader.Create(new System.IO.StringReader(Dashboard));
        }

        public static System.Xml.XmlReader DashboardToXml(byte[] Dashboard)
        {
            return DashboardToXml(DashboardToString(Dashboard));
        }

        public static string DashboardToString(byte[] Dashboard)
        {
            string cleanedDashboard = null;

            if (Dashboard != null)
            {
                cleanedDashboard = System.Text.Encoding.UTF8.GetString(Dashboard, 0, Dashboard.Length);
                cleanedDashboard = cleanedDashboard.Replace("\n", String.Empty);
                cleanedDashboard = cleanedDashboard.Replace("\t", String.Empty);
                cleanedDashboard = cleanedDashboard.Substring(0, cleanedDashboard.IndexOf('\0'));
            }

            return cleanedDashboard;
        }

        public static string XmlToDashboard(System.Xml.XmlReader Dashboard)
        {
            return Dashboard.ToString();
        }

        public static Objects.NXTEntity DashboardToNXTEntity(string dashboard)
        {
            Objects.NXTEntity lunar = new Objects.NXTEntity();
            System.Xml.XmlReader reader = DashboardToXml(dashboard);

            while (reader.Read())
            {
                if (reader.NodeType == System.Xml.XmlNodeType.Element)
                {
                    switch (reader.Name)
                    {
                        case "wheels":
                            lunar = DashboardToNXTEntityWheels(lunar, reader);
                            break;
                        case "wheel":
                            lunar = DashboardToNXTEntityWheel(lunar, reader);
                            break;
                        case "ultrasonics":
                            lunar = DashboardToNXTEntityUltrasonics(lunar, reader);
                            break;
                        case "ultrasonic":
                            lunar = DashboardToNXTEntityUltrasonic(lunar, reader);
                            break;
                        case "colors":
                            lunar = DashboardToNXTEntityColors(lunar, reader);
                            break;
                        case "color":
                            lunar = DashboardToNXTEntityColor(lunar, reader);
                            break;
                        case "contacts":
                            lunar = DashboardToNXTEntityContacts(lunar, reader);
                            break;
                        case "contact":
                            lunar = DashboardToNXTEntityContact(lunar, reader);
                            break;
                    }
                }
            }

            return lunar;
        }

        private static Objects.NXTEntity DashboardToNXTEntityWheels(Objects.NXTEntity nxtEntity, System.Xml.XmlReader reader)
        {
            Objects.NXTEntity newEntity = nxtEntity;
            Objects.NXTDashboard_Wheel newWheel = new Objects.NXTDashboard_Wheel();
            string wheelType = null;
            int wheelsDepth = 0;
            int wheelDepth = 0;

            if (reader.IsStartElement("wheels"))
            {
                wheelsDepth = reader.Depth;
                reader.Read();
                while (reader.Depth > wheelsDepth)
                {
                    if (reader.IsStartElement("wheel"))
                    {
                        wheelType = reader.GetAttribute("type");
                        wheelDepth = reader.Depth;
                        reader.Read();
                        while (reader.Depth > wheelDepth)
                        {
                            switch (reader.Name)
                            {
                                case "name":
                                    newWheel.Name = reader.GetAttribute("value");
                                    break;
                                case "effort":
                                    newWheel.Effort = Int32.Parse(reader.GetAttribute("value"));
                                    break;
                                case "position":
                                    newWheel.Position = Int32.Parse(reader.GetAttribute("value"));
                                    break;
                                case "velocity":
                                    newWheel.Velocity = Int32.Parse(reader.GetAttribute("value"));
                                    break;
                            }
                            reader.Read();
                        }
                        if (wheelType == "right")
                        {
                            newEntity.RightWheel = newWheel;
                        }
                        else if (wheelType == "left")
                        {
                            newEntity.LeftWheel = newWheel;
                        }
                        else
                        {
                            newEntity.AuxWheel = newWheel;
                        }
                        newWheel = new Objects.NXTDashboard_Wheel();
                    }
                    reader.Read();
                }
            }

            return newEntity;
        }

        private static Objects.NXTEntity DashboardToNXTEntityWheel(Objects.NXTEntity nxtEntity, System.Xml.XmlReader reader)
        {
            Objects.NXTEntity newEntity = nxtEntity;

            //TODO : (:

            return newEntity;
        }

        private static Objects.NXTEntity DashboardToNXTEntityUltrasonics(Objects.NXTEntity nxtEntity, System.Xml.XmlReader reader)
        {
            Objects.NXTEntity newEntity = nxtEntity;
            Objects.NXTDashboard_Ultrasonic newUltrasonic = new Objects.NXTDashboard_Ultrasonic();

            //TODO : (:

            return newEntity;
        }

        private static Objects.NXTEntity DashboardToNXTEntityUltrasonic(Objects.NXTEntity nxtEntity, System.Xml.XmlReader reader)
        {
            Objects.NXTEntity newEntity = nxtEntity;
            Objects.NXTDashboard_Ultrasonic newUltrasonic = new Objects.NXTDashboard_Ultrasonic();
            int ultrasonicDepth = 0;

            if (reader.IsStartElement("ultrasonic"))
            {
                ultrasonicDepth = reader.Depth;
                reader.Read();
                while (reader.Depth > ultrasonicDepth)
                {
                    switch (reader.Name)
                    {
                        case "range":
                            newUltrasonic.Range = Int32.Parse(reader.GetAttribute("value"));
                            break;
                        case "rangeMin":
                            newUltrasonic.RangeMin = Int32.Parse(reader.GetAttribute("value"));
                            break;
                        case "rangeMax":
                            newUltrasonic.RangeMax = Int32.Parse(reader.GetAttribute("value"));
                            break;
                        case "spreadAngle":
                            newUltrasonic.SpreadAngle = Int32.Parse(reader.GetAttribute("value"));
                            break;
                    }
                    reader.Read();
                }
                nxtEntity.Ultrasonic = newUltrasonic;
            }

            return newEntity;
        }

        private static Objects.NXTEntity DashboardToNXTEntityColors(Objects.NXTEntity nxtEntity, System.Xml.XmlReader reader)
        {
            Objects.NXTEntity newEntity = nxtEntity;
            Objects.NXTDashboard_Color newColor = new Objects.NXTDashboard_Color();

            //TODO : (:

            return newEntity;
        }

        private static Objects.NXTEntity DashboardToNXTEntityColor(Objects.NXTEntity nxtEntity, System.Xml.XmlReader reader)
        {
            Objects.NXTEntity newEntity = nxtEntity;
            Objects.NXTDashboard_Color newColor = new Objects.NXTDashboard_Color();
            int colorDepth = 0;

            if (reader.IsStartElement("color"))
            {
                colorDepth = reader.Depth;
                reader.Read();
                while (reader.Depth > colorDepth)
                {
                    switch (reader.Name)
                    {
                        case "intensity":
                            newColor.Intensity = Int32.Parse(reader.GetAttribute("value"));
                            break;
                        case "red":
                            newColor.Red = Int32.Parse(reader.GetAttribute("value"));
                            break;
                        case "green":
                            newColor.Green = Int32.Parse(reader.GetAttribute("value"));
                            break;
                        case "blue":
                            newColor.Blue = Int32.Parse(reader.GetAttribute("value"));
                            break;
                    }
                    reader.Read();
                }
                nxtEntity.Color = newColor;
            }

            return newEntity;
        }

        private static Objects.NXTEntity DashboardToNXTEntityContacts(Objects.NXTEntity nxtEntity, System.Xml.XmlReader reader)
        {
            Objects.NXTEntity newEntity = nxtEntity;
            int contactsDepth = 0;

            if (reader.IsStartElement("contacts"))
            {
                contactsDepth = reader.Depth;
                reader.Read();
                while (reader.Depth > contactsDepth)
                {
                    if (reader.IsStartElement("contact"))
                    {
                        if (reader.GetAttribute("type") == "right")
                        {
                            newEntity.RightContact.IsTouching = bool.Parse(reader.GetAttribute("value"));
                        }
                        else if (reader.GetAttribute("type") == "left")
                        {
                            newEntity.LeftContact.IsTouching = bool.Parse(reader.GetAttribute("value"));
                        }
                    }
                    reader.Read();
                }
            }

            return newEntity;
        }

        private static Objects.NXTEntity DashboardToNXTEntityContact(Objects.NXTEntity nxtEntity, System.Xml.XmlReader reader)
        {
            Objects.NXTEntity newEntity = nxtEntity;
            Objects.NXTDashboard_Contact newContact = new Objects.NXTDashboard_Contact();

            //TODO : (:

            return newEntity;
        }


        public static string SendOrder(NXTRemote.Objects.NXTOrder order)
        {
            string xmlOrder = SendOrderXMLSkeleton();

            xmlOrder = xmlOrder.Replace("OrderValueToReplace", order.Order);
            xmlOrder = xmlOrder.Replace("EffortValueToReplace", order.Effort.ToString());
            xmlOrder = xmlOrder.Replace("DirectionValueToReplace", order.Direction.ToString());

            return xmlOrder;
        }

        public static string SendOrder(string order, string effort, string effortMultiplicator, string direction)
        {
            string xmlOrder = SendOrderXMLSkeleton();

            if (order != "0" && order != "1" && NXTRemote.Controllers.XML.isStringAnotherType(effort, System.TypeCode.Int32) == true && NXTRemote.Controllers.XML.isStringAnotherType(direction, System.TypeCode.Boolean) == true)
            {
                // Put effort to 0 if there is any incorrect value
                order = "0";
                effort = "0";
                effortMultiplicator = "0";
                direction = "true";
            }

            xmlOrder = xmlOrder.Replace("OrderValueToReplace", order);
            xmlOrder = xmlOrder.Replace("EffortValueToReplace", (Int32.Parse(effort) * Int32.Parse(effortMultiplicator)).ToString());
            xmlOrder = xmlOrder.Replace("DirectionValueToReplace", direction);

            return xmlOrder;
        }

        private static string SendOrderXMLSkeleton()
        {
            return @"<?xml version=""1.0"" encoding=""UTF-8""?><order><order value=""OrderValueToReplace"" /><effort value=""EffortValueToReplace"" /><direction value=""DirectionValueToReplace"" /></order>";
        }

        /* public static NXTRemote.Objects.NXTEntity DashboardToEntity(string dashboard)
        {
            NXTRemote.Objects.NXTEntity entity = new NXTRemote.Objects.NXTEntity();
            System.Xml.Linq.XElement xDashboard = System.Xml.Linq.XElement.Load(dashboard);

            System.Collections.Generic.List<NXTRemote.Objects.NXTDashboard_Color> colorList = new System.Collections.Generic.List<Objects.NXTDashboard_Color>();
            System.Collections.Generic.List<NXTRemote.Objects.NXTDashboard_Contact> contactList = new System.Collections.Generic.List<Objects.NXTDashboard_Contact>();
            System.Collections.Generic.List<NXTRemote.Objects.NXTDashboard_Ultrasonic> ultrasonicList = new System.Collections.Generic.List<Objects.NXTDashboard_Ultrasonic>();
            System.Collections.Generic.List<NXTRemote.Objects.NXTDashboard_Wheel> wheelList = new System.Collections.Generic.List<Objects.NXTDashboard_Wheel>();

            foreach (System.Xml.Linq.XElement xEl in xDashboard.Elements("dashboard"))
            {
                if (xEl.Name == "color")
                {
                }
                else if (xEl.Name == "contacts")
                {
                }
                else if (xEl.Name == "ultrasonic")
                {
                }
                else if (xEl.Name == "wheels")
                {
                }
            }

            return entity;
        } */

        public static bool isStringAnotherType(string valueToCheck, TypeCode typeToCheck)
        {
            bool result = false;

            if (typeToCheck == System.TypeCode.Boolean)
            {
                Boolean booleanCheck;
                result = Boolean.TryParse(valueToCheck, out booleanCheck);
            }
            else if (typeToCheck == System.TypeCode.Char)
            {
                Char charCheck;
                result = Char.TryParse(valueToCheck, out charCheck);
            }
            else if (typeToCheck == System.TypeCode.Double)
            {
                Double doubleCheck;
                result = Double.TryParse(valueToCheck, out doubleCheck);
            }
            else if (typeToCheck == System.TypeCode.Int32)
            {
                Int32 int32Check;
                result = Int32.TryParse(valueToCheck, out int32Check);
            }
            else if (typeToCheck == System.TypeCode.Int64)
            {
                Int64 int64Check;
                result = Int64.TryParse(valueToCheck, out int64Check);
            }
            else if (typeToCheck == System.TypeCode.Single)
            {
                Single singleCheck;
                result = Single.TryParse(valueToCheck, out singleCheck);
            }
            else if (typeToCheck == System.TypeCode.String)
            {
                result = true;
            }
            else
            {
                result = false;
            }

            return result;
        }

    }
}
