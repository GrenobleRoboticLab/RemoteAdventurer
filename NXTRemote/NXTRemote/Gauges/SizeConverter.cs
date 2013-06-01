using System;

namespace NXTRemote.Gauges
{
    public class SizeConverter : System.Windows.Data.IValueConverter
    {
        public double StartSize { get; set; }
        public double EndSize { get; set; }
        public double DefaultSize { get; set; }
        public double StartValue { get; set; }
        public double EndValue { get; set; }
        public SizeConverter()
        {
            StartSize = EndSize = DefaultSize = 10;
        }
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            double val = (double)value;
            double size = DefaultSize;
            if (val >= StartValue && val <= EndValue && EndValue != StartValue)
            {
                size = StartSize + (EndSize - StartSize) * val / (EndValue - StartValue);
            }
            return size;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
