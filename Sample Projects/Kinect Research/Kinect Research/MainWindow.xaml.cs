using System.Windows;
using Microsoft.Kinect;         // Used by our Kinect API (namespace)
using System.ComponentModel;
using System;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Controls;
using System.Windows.Documents;
using Coding4Fun.Kinect.Wpf.Controls;
using System.Windows.Shapes;
using System.Linq;
using System.Collections.Generic;

namespace KinectResearch
{
    public partial class MainWindow : Window
    {
        private KinectSensor kinect = null;                 // Kinect Device Variable
        private ColorFrameReader colorFrameReader = null;   // Read frames from the kinect
        private WriteableBitmap colorBitmap = null;         // Object used to modify a set of pixels dynamically
        private MultiSourceFrameReader multiSourceFrameReader;
        //private BodyFrameReader bodyFrameReader;
        private CoordinateMapper coordinateMapper;
        FrameDescription colorFrameDescription;
        private bool dataReceived;
        private Body[] bodies;
        private List<Tuple<JointType, JointType>> bones;
        private List<SolidColorBrush> bodyColors;

        public MainWindow()
        {
            bones = new List<Tuple<JointType, JointType>>();
            //Torso
            bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));
            //Left Arm
            bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));
            //Right Arm
            bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));
            //Left Leg
            bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
            //Right Leg
            bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            bodyColors = new List<SolidColorBrush>();
            bodyColors.Add(new SolidColorBrush(Colors.Red));
            bodyColors.Add(new SolidColorBrush(Colors.Green));
            bodyColors.Add(new SolidColorBrush(Colors.Orange));
            bodyColors.Add(new SolidColorBrush(Colors.Blue));
            bodyColors.Add(new SolidColorBrush(Colors.Yellow));
            bodyColors.Add(new SolidColorBrush(Colors.Pink));

            //remaining code in constructor
            kinect = KinectSensor.GetDefault();
            multiSourceFrameReader = kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color |
           FrameSourceTypes.Body);
            multiSourceFrameReader.MultiSourceFrameArrived += MultiSourceFrameReader_MultiSourceFrameArrived;
            coordinateMapper = kinect.CoordinateMapper;
            colorFrameDescription = kinect.ColorFrameSource.CreateFrameDescription
            (ColorImageFormat.Bgra);
            colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.
            Height, 96.0, 96.0, PixelFormats.Bgra32, null);
            kinect.Open();
            DataContext = this;
            InitializeComponent();

        }
        private void MultiSourceFrameReader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        colorBitmap.Lock();

                    if ((colorFrameDescription.Width == colorBitmap.PixelWidth) &&
                            (colorFrameDescription.Height == colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                            colorBitmap.BackBuffer,
                            (uint)(colorFrameDescription.Width *
                           colorFrameDescription.Height * 4),
                            ColorImageFormat.Bgra);
                            colorBitmap.AddDirtyRect(new Int32Rect(0, 0, colorBitmap.PixelWidth,
                           colorBitmap.PixelHeight));
                        }
                        colorBitmap.Unlock();
                    }
                }
            }
            using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (bodies == null)
                    {
                        bodies = new Body[bodyFrame.BodyCount];
                    }
                    bodyFrame.GetAndRefreshBodyData(bodies);
                    dataReceived = true;
                }
            }
            if (dataReceived)
            {
                canvas.Children.Clear();

                //Add this line
                int colorIndex = 0;

                foreach (Body body in bodies.Where(b => b.IsTracked))
                {
                    //Add the following lines
                    SolidColorBrush colorBrush = bodyColors[colorIndex++];
                    Dictionary<JointType, Point> jointColorPoints = new Dictionary<JointType, Point>();

                    foreach (var joint in body.Joints)
                    {
                        CameraSpacePoint position = joint.Value.Position;
                        if (position.Z < 0)
                        {
                            position.Z = 0.1f;
                        }
                        ColorSpacePoint colorSpacePoint = coordinateMapper.MapCameraPointToColorSpace
                       (position);

                        //Add this line
                        jointColorPoints[joint.Key] = new Point(colorSpacePoint.X, colorSpacePoint.Y);

                        if (joint.Value.TrackingState == TrackingState.Tracked)
                        {
                            DrawJoint(new Point(colorSpacePoint.X, colorSpacePoint.Y), new
                           SolidColorBrush(Colors.Purple));
                        }
                        if (joint.Value.TrackingState == TrackingState.Inferred)
                        {
                            DrawJoint(new Point(colorSpacePoint.X, colorSpacePoint.Y), new
                           SolidColorBrush(Colors.LightGray));
                        }
                        foreach (var bone in bones)
                        {
                            DrawBone(body.Joints, jointColorPoints, bone.Item1, bone.Item2, colorBrush);
                        }
                    }
                }
            }
        }
        private void DrawJoint(Point jointCoord, SolidColorBrush s)
        {
            if (jointCoord.X < 0 || jointCoord.Y < 0)
                return;
            Ellipse ellipse = new Ellipse()
            {
                Width = 10,
                Height = 10,
                Fill = s
            };
            Canvas.SetLeft(ellipse, (jointCoord.X / colorFrameDescription.Width) * canvas.ActualWidth - ellipse.Width / 2);
            Canvas.SetTop(ellipse, (jointCoord.Y / colorFrameDescription.Height) * canvas.ActualHeight - ellipse.Height / 2);
            canvas.Children.Add(ellipse);
        }
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointColorPoints, JointType jointType0, JointType jointType1, SolidColorBrush color)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];
            if (joint0.TrackingState == TrackingState.NotTracked || joint1.TrackingState ==
           TrackingState.NotTracked)
                return;
            if (jointColorPoints[jointType0].X < 0 || jointColorPoints[jointType0].Y < 0 ||
           jointColorPoints[jointType1].X < 0 || jointColorPoints[jointType0].Y < 0)
                return;
            Line line = new Line()
            {
                X1 = (jointColorPoints[jointType0].X / colorFrameDescription.Width) * canvas.ActualWidth,
                Y1 = (jointColorPoints[jointType0].Y / colorFrameDescription.Height) * canvas.ActualHeight,
                X2 = (jointColorPoints[jointType1].X / colorFrameDescription.Width) * canvas.ActualWidth,
                Y2 = (jointColorPoints[jointType1].Y / colorFrameDescription.Height) * canvas.ActualHeight,
                StrokeThickness = 5,
                Stroke = color
            };
            canvas.Children.Add(line);
        }
        public ImageSource ImageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();    // Mutex
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) &&
                       (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                            this.colorBitmap.BackBuffer,
                            (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                            ColorImageFormat.Bgra);
                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.
                           PixelWidth, this.colorBitmap.PixelHeight));
                        }
                        this.colorBitmap.Unlock();
                    }
                }
            }
        }
        private void button1_Click(object sender, RoutedEventArgs e)
        {
            MessageBox.Show("Button 1 is clicked!");
        }
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }
            if (this.kinect != null)
            {
                this.kinect.Close();
                this.kinect = null;
            }
        }
    }
}
