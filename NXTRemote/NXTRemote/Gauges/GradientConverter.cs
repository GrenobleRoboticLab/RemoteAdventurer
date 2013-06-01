using System;

namespace NXTRemote.Gauges
{
    public class GradientConverter : System.Windows.Data.IValueConverter
    {
        public System.Windows.Media.Color StartColor { get; set; }
        public System.Windows.Media.Color EndColor { get; set; }
        public System.Windows.Media.Color DefaultColor { get; set; }
        public double StartValue { get; set; }
        public double EndValue { get; set; }
        public GradientConverter()
        {
            DefaultColor = System.Windows.Media.Colors.White;
            StartColor = System.Windows.Media.Colors.White;
            EndColor = System.Windows.Media.Colors.White;
        }
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            double val = (double)value;
            int n = (int)(EndValue - StartValue);
            int curr = (int)(val - StartValue);
            System.Windows.Media.Color newColor = DefaultColor;
            if (val >= StartValue && val <= EndValue)
            {
                double u = (double)curr / n;
                newColor.R = (byte)(StartColor.R * (1 - u) + EndColor.R * u);
                newColor.G = (byte)(StartColor.G * (1 - u) + EndColor.G * u);
                newColor.B = (byte)(StartColor.B * (1 - u) + EndColor.B * u);
            }
            return new System.Windows.Media.SolidColorBrush(newColor);
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
