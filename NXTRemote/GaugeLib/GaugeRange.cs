
using System.Windows.Media;
using System.ComponentModel;
namespace GaugeLib
{
    public class GaugeRange:INotifyPropertyChanged
    {
        private double offset;
        private Color color;

        public double Offset
        {
            get { return offset; }
            set
            {
                if (offset != value)
                {
                    offset = value;
                    OnNotifyPropertyChanged("Offset");
                }
            }
        }
        public Color Color
        {
            get { return color; }
            set
            {
                if (color != value)
                {
                    color = value;
                    OnNotifyPropertyChanged("Color");
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;
        private void OnNotifyPropertyChanged(string name)
        {
            if (PropertyChanged != null)
                PropertyChanged(this, new PropertyChangedEventArgs(name)); 
        }
    }
}
