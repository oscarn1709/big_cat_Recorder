using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.Diagnostics;
using Newtonsoft.Json;
using System.IO;
using System.Net;
using LightBuzz.Vitruvius;
using System.Collections.ObjectModel;

namespace KinectGUI
{
    public class JointPackage
    {
        public Joint [] jointsArray;
        public double stamp;

        public JointPackage(Joint[] j, double s)
        {
            this.jointsArray = j;
            this.stamp = s;
        }
    }
    
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public double maxHeight = 0;
        public double stampHeight = 0;
        public int flag = 0;

        List<JointPackage> sendToServer = new List<JointPackage>();

        Mode _mode = Mode.Color;
   
        KinectSensor kinect;
        MultiSourceFrameReader reader;
        IList<Body> bodies;

        bool trackBodies = false;


        public MainWindow()
        {
            InitializeComponent();
            Loaded += OnLoaded;
        }

        

        // On GUI Load
        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            
            // Kinect sensor is found and opened
            this.kinect = KinectSensor.GetDefault();
            if( kinect != null)
            {
                this.kinect.Open();
            }
            

            // Frame reader 
            this.reader = this.kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color |
                                                                 FrameSourceTypes.Depth |
                                                                 FrameSourceTypes.Body);
            this.reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // Create a reference to the MultiSource Frame
            var reference = e.FrameReference.AcquireFrame();

            // Open the color Frame
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                // Check for the null frame
                if( frame != null)
                {
                    if( _mode == Mode.Color)
                        camera.Source = ProcessFrame(frame);
                }

            }

            // Open the depth frame
            using (var frame = reference.DepthFrameReference.AcquireFrame())
            {
                
                if( frame != null)
                {
                    if( _mode == Mode.Depth)
                        camera.Source = ProcessFrame(frame);
                }
            }

            // Open the body frame
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if( frame != null)
                {
                    canvas.Children.Clear();

                    if(trackBodies)
                    {
                        // Create a new body object to store the tracked bodies in kinect
                        bodies = new Body[frame.BodyFrameSource.BodyCount];
                        //Trace.WriteLine("Number of bodies = " + bodies.Count);
                        frame.GetAndRefreshBodyData(bodies);
                        //Trace.WriteLine(bodies[0].ToString());
                        foreach( var body in bodies)
                        {
                           // Trace.WriteLine( body == null ? "Null == Yes" : "Null == No");
                            if(body != null)
                            {
                              //  Trace.WriteLine(body.IsTracked ? "Tracked == Yes" : "Tracked == No");
                                if (body.IsTracked)
                                {
                                    /*
                                     * Once the record button is pressed, the record function will run each frame
                                     */
                                    RecordJSON(body);
                                }

                            }
                        }

                    }
                }
            }
        }

        private void CheckSymmetry(Body body)
        {
            //Knee
            Joint hipR = body.Joints[JointType.HipRight];
            Joint kneeR = body.Joints[JointType.KneeRight];
            Joint ankleR = body.Joints[JointType.AnkleRight];

            double angleKneeRight = kneeR.Angle(ankleR, hipR);

            Joint hipL = body.Joints[JointType.HipLeft];
            Joint kneeL = body.Joints[JointType.KneeLeft];
            Joint ankleL = body.Joints[JointType.AnkleLeft];

            double angleKneeLeft = kneeL.Angle(hipL, ankleL);
            Trace.WriteLine(angleKneeRight + " " + angleKneeLeft);

            if (angleKneeRight > 270 && angleKneeLeft > 270)
                Tester.Text = "HIT DEPTH";
            else
                Tester.Text = "";
        }

        private void RecordJSON(Body body)
        {
            //Height of user is the Y position of head joint
            double Height = body.Joints[JointType.Head].Position.Y;
           
            if (Height > this.maxHeight)
            {
                if(flag == 0)
                {
                    this.maxHeight = Height;
                    stampHeight = this.maxHeight;
                    flag = 1;
                }
                else if(Math.Abs(Height - this.maxHeight) >= 0.1)
                {
                    Tester.Text = "BREAK";
                }
            }
                
             if (stampHeight - Height >= 0.15)
             {
                stampHeight = Height;

                Joint[] myJoints = new Joint[body.Joints.Count];
                int i = 0;
                foreach(Joint joint in body.Joints.Values)
                {
                    myJoints[i++] = joint;
                }

                // index = MaxHeight - stampHeight
                double index = Math.Round(this.maxHeight - stampHeight, 3);

                JointPackage jp = new JointPackage(myJoints, index);
                string json = JsonConvert.SerializeObject(jp);
                Trace.WriteLine(json);

                sendToServer.Add(jp);
             }

             if (Height - stampHeight >= -0.1)
             {
                Tester.Text = "BREAK";
             }
            
        }


        private void DrawPoint(Canvas canvas, Joint joint)
        {
            if (joint.TrackingState == TrackingState.NotTracked) return;
            

            // Map the joint to the pixels
            joint = ScaleTo(joint, canvas.ActualWidth, canvas.ActualHeight);

            // Create an elipse for the joint
            Ellipse ellipse = new Ellipse
            {
                Width = 20,
                Height = 20,
                Fill = new SolidColorBrush(Colors.Gold)
            };

            // Sets the offset Values for the Joint dots relative to the Canvas
            Canvas.SetLeft(ellipse, joint.Position.X - (ellipse.Width / 2));
            Canvas.SetTop(ellipse, joint.Position.Y - (ellipse.Height / 2));

            canvas.Children.Add(ellipse);
        }

        private void DrawLine(Canvas canvas, Joint first, Joint second)
        {
            if (first.TrackingState == TrackingState.NotTracked || second.TrackingState == TrackingState.NotTracked) return;

            // Scale the joints to the canvas
            first = ScaleTo(first, canvas.ActualWidth, canvas.ActualHeight);
            second = ScaleTo(second, canvas.ActualWidth, canvas.ActualHeight);

            // Create a Line object to connect them 
            Line line = new Line
            {
                X1 = first.Position.X,
                Y1 = first.Position.Y,
                X2 = second.Position.X,
                Y2 = second.Position.Y,
                StrokeThickness = 8,
                Stroke = new SolidColorBrush(Colors.Black)
            };

            canvas.Children.Add(line);


        }


        private void DrawBody(Canvas canvas, Body body)
        {
            if (body == null) return;

            foreach(Joint joint in body.Joints.Values)
            {
                DrawPoint(canvas, joint);
                string jsonTest = JsonConvert.SerializeObject(joint);
                File.WriteAllText(path: @"C:\Users\oscar\Desktop\OutputTest.json", contents: DateTime.Now.ToString(format: "h:mm:ss tt"));
                File.WriteAllText(@"C:\Users\oscar\Desktop\OutputTest.json", jsonTest);
                //Trace.WriteLine("Position Values for the " + joint.JointType.ToString() + " Joint\n" + 
                //                "X Value= " + joint.Position.X + "\nY Value = " + joint.Position.Y + "\nZ Position = " + joint.Position.Z);
            }

            // Manually draw the lines between all the joint connections. 
            DrawLine(canvas, body.Joints[JointType.Head], body.Joints[JointType.Neck]);
            DrawLine(canvas, body.Joints[JointType.Neck], body.Joints[JointType.SpineShoulder]);
            DrawLine(canvas, body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderRight]);
            DrawLine(canvas, body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderLeft]);
            DrawLine(canvas, body.Joints[JointType.SpineShoulder], body.Joints[JointType.SpineMid]);
            DrawLine(canvas, body.Joints[JointType.SpineMid], body.Joints[JointType.SpineBase]);
            DrawLine(canvas, body.Joints[JointType.ShoulderLeft], body.Joints[JointType.ElbowLeft]);
            DrawLine(canvas, body.Joints[JointType.ShoulderRight], body.Joints[JointType.ElbowRight]);
            DrawLine(canvas, body.Joints[JointType.ElbowRight], body.Joints[JointType.WristRight]);
            DrawLine(canvas, body.Joints[JointType.ElbowLeft], body.Joints[JointType.WristLeft]);
            DrawLine(canvas, body.Joints[JointType.WristLeft], body.Joints[JointType.HandLeft]);
            DrawLine(canvas, body.Joints[JointType.WristRight], body.Joints[JointType.HandRight]);
            DrawLine(canvas, body.Joints[JointType.HandLeft], body.Joints[JointType.HandTipLeft]);
            DrawLine(canvas, body.Joints[JointType.HandRight], body.Joints[JointType.HandTipRight]);
            DrawLine(canvas, body.Joints[JointType.HandTipLeft], body.Joints[JointType.ThumbLeft]);
            DrawLine(canvas, body.Joints[JointType.HandTipRight], body.Joints[JointType.ThumbRight]);
            DrawLine(canvas, body.Joints[JointType.SpineBase], body.Joints[JointType.HipLeft]);
            DrawLine(canvas, body.Joints[JointType.SpineBase], body.Joints[JointType.HipRight]);
            DrawLine(canvas, body.Joints[JointType.HipLeft], body.Joints[JointType.KneeLeft]);
            DrawLine(canvas, body.Joints[JointType.HipRight], body.Joints[JointType.KneeRight]);
            DrawLine(canvas, body.Joints[JointType.KneeLeft], body.Joints[JointType.AnkleLeft]);
            DrawLine(canvas, body.Joints[JointType.KneeRight], body.Joints[JointType.AnkleRight]);
            DrawLine(canvas, body.Joints[JointType.AnkleLeft], body.Joints[JointType.FootLeft]);
            DrawLine(canvas, body.Joints[JointType.AnkleRight], body.Joints[JointType.FootRight]);
        }

        /*  The following few functions are the courtesy of Vangos Pternas 
         *  https://pterneas.com/2014/02/20/kinect-for-windows-version-2-color-depth-and-infrared-streams/
         *  They serve as the bitMap processing for the frames
         *  and the scaling of the skeleton to the pixel frame as well.
         */
          private ImageSource ProcessFrame(DepthFrame frame)
        {
            // Determining the size of the incoming frames
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;

            // Established the pixel format, min and max depth distances
            PixelFormat format = PixelFormats.Bgr32;
            ushort minDepth = frame.DepthMinReliableDistance;
            ushort maxDepth = frame.DepthMaxReliableDistance;

            // Creates a pixel data array that will store the pixel data for the frame 
            ushort[] pixelData = new ushort[width * height];

            // the output pixel map for the depth image
            byte[] pixels = new byte[width * height * (format.BitsPerPixel + 7) / 8];

            // Copies frame data to the PixelData array 
            frame.CopyFrameDataToArray(pixelData);

            int colorIndex = 0;
            // Goes through all the Pixels in the PixelData array
            for (int depthIndex = 0; depthIndex < pixelData.Length; depthIndex++)
            {
                ushort depth = pixelData[depthIndex];

                byte intensity = (byte)((depth >= minDepth && depth <= maxDepth) ? depth : 0);
                pixels[colorIndex++] = intensity; // Blue
                pixels[colorIndex++] = intensity; // Green
                pixels[colorIndex++] = intensity; // Red
                colorIndex++;
            }
            int stride = width * format.BitsPerPixel / 8;
            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

          private ImageSource ProcessFrame(ColorFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;

            PixelFormat format = PixelFormats.Bgr32;

            byte[] pixels = new byte[width * height * ((format.BitsPerPixel + 7) / 8)];

            if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                frame.CopyRawFrameDataToArray(pixels);
            }
            else
            {
                frame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        /*
        *  Scaling Functions
        */
        public static Joint ScaleTo(Joint joint, double width, double height, float skeletonMaxX, float skeletonMaxY)
        {
            joint.Position = new CameraSpacePoint
            {
                X = Scale(width, skeletonMaxX, joint.Position.X),
                Y = Scale(height, skeletonMaxY, -joint.Position.Y),
                Z = joint.Position.Z
            };
            return joint;
        }

        public static Joint ScaleTo(Joint joint, double width, double height)
        {
            return ScaleTo(joint, width, height, 1.0f, 1.0f);
        }

        private static float Scale(double maxPixel, double maxSkeleton, float position)
        {
            float value = (float)((((maxPixel / maxSkeleton) / 2) * position) + (maxPixel / 2));

            if (value > maxPixel)
            {
                return (float)maxPixel;
            }

            if (value < 0)
            {
                return 0;
            }

            return value;
        }



        /* 
         * Button Handlers
         */

        private void Color_Click(object sender,RoutedEventArgs e)
        {
            _mode = Mode.Color; 
        }
        private void Depth_Click(object sender, RoutedEventArgs e)
        {
            _mode = Mode.Depth;
        }
        private void Body_Click(object sender, RoutedEventArgs e)
        {
            trackBodies = !trackBodies;
        }
        private void Record_Click(object sender, RoutedEventArgs e)
        {
            trackBodies = !trackBodies;
        }
        private void Angle_Click(object sender, RoutedEventArgs e)
        {
            trackBodies = !trackBodies;
        }


        // Program end
        private void Window_Closed(object sender, EventArgs e)
        {
            if(reader != null)
            {
                reader.Dispose();
            }
            if(kinect != null)
            {
                kinect.Close();
            }
        }

    }

    public enum Mode
    {
        Color, Depth
    }
}
