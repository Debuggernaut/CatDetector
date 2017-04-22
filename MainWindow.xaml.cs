// Dear DSP and/or math experts:  Please have mercy on me; I am but a humble
// kernel driver developer

namespace com.HardModeCode.CatDetector
{
    using System;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Collections.Generic;
    using System.Media;    /// <summary>
                           /// Interaction logic for MainWindow.xaml
                           /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap colorBitmap;

        /// <summary>
        /// Intermediate storage for the depth data received from the camera
        /// </summary>
        private DepthImagePixel[] depthPixels;

        /// <summary>
        /// Baseline for the area
        /// </summary>
        private DepthImagePixel[] depthBaseline;

        private int[] diffMap;
        private int[] diffMapScratch;

        private Stack<int> tempStack1;
        private Stack<int> tempStack2;

        /// <summary>
        /// Indicates that the system is armed and will sound an alarm
        /// upon the detection of a moving target
        /// </summary>
        private bool systemArmed = false;

        /// <summary>
        /// Indicates that the alarm has been triggered and is currently playing
        /// </summary>
        private bool alarmIsPlaying = false;

        /// <summary>
        /// Flag set by to indicate that the next depth frame should be
        /// saved as the new baseline
        /// </summary>
        private bool captureBaseline = false;

        /// <summary>
        /// Beyond this threshold, a change in depth is assumed
        /// to be a possible cat or VR user
        /// </summary>
        private int depthThreshold = 50;

        /// <summary>
        /// There must be at least this many contiguous pixels
        /// past the threshold to be considered a new source of
        /// movement
        /// </summary>
        private int targetSizeThreshold=800;

        /// <summary>
        /// Filter out narrow shapes like the headset cable by
        /// growing the background by this many pixels
        /// movement
        /// </summary>
        private int borderGrowth = 5;

        /// <summary>
        /// Intermediate storage for the depth data converted to color
        /// </summary>
        private byte[] colorPixels;
        
        private SoundPlayer Player = new SoundPlayer();

        //Which: 0,1,2,3 = N E S W
        private int getNode(int center, int whichDirection)
        {
            switch (whichDirection)
            {
                case 0:
                    if (center - this.sensor.DepthStream.FrameWidth < 0)
                        return -1;
                    else
                        return center - this.sensor.DepthStream.FrameWidth;
                case 1:
                    if ((center % this.sensor.DepthStream.FrameWidth) + 1 >= this.sensor.DepthStream.FrameWidth)
                        return -1;
                    else
                        return center + 1;
                case 2:
                    if (center + this.sensor.DepthStream.FrameWidth >= this.sensor.DepthStream.FramePixelDataLength)
                        return -1;
                    else
                        return center + this.sensor.DepthStream.FrameWidth;
                case 3:
                    if ((center % this.sensor.DepthStream.FrameWidth) - 1 <= 0)
                        return -1;
                    else
                        return center - 1;
                default:
                    return -1;
            }
        }

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the depth stream to receive depth frames
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                this.systemArmed = false;

                // Allocate space to put the depth pixels we'll receive
                this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
                this.depthBaseline = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];

                this.diffMap = new int[this.sensor.DepthStream.FramePixelDataLength];
                this.diffMapScratch = new int[this.sensor.DepthStream.FramePixelDataLength];

                tempStack1 = new Stack<int>(this.sensor.DepthStream.FramePixelDataLength);
                tempStack2 = new Stack<int>(this.sensor.DepthStream.FramePixelDataLength);

                // Allocate space to put the color pixels we'll create
                this.colorPixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                this.Image.Source = this.colorBitmap;

                // Add an event handler to be called whenever there is new depth frame data
                this.sensor.DepthFrameReady += this.SensorDepthFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's DepthFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {   
                    if (this.captureBaseline)
                    {
                        depthFrame.CopyDepthImagePixelDataTo(this.depthBaseline);
                        this.captureBaseline = false;
                        this.systemArmed = true;
                        return;
                    }

                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);

                    // Get the min and max reliable depth for the current frame
                    int minDepth = depthFrame.MinDepth;
                    int maxDepth = depthFrame.MaxDepth;

                    // Convert the depth to RGB
                    int colorPixelIndex = 0;
                    for (int i = 0; i < this.depthPixels.Length; ++i)
                    {
                        // Get the depth for this pixel
                        short depth = depthPixels[i].Depth;

                        this.diffMap[i] = 0;

                        if (depth >= minDepth && depth <= maxDepth)
                        {
                            if (depthBaseline[i].Depth - depth > depthThreshold)
                            {
                                this.diffMap[i] = 1;
                                
                                // Write out blue byte
                                this.colorPixels[colorPixelIndex++] = 0;

                                // Write out green byte
                                this.colorPixels[colorPixelIndex++] = 255;

                                // Write out red byte                        
                                this.colorPixels[colorPixelIndex++] = 0;
                            }
                            else
                            {
                                byte intensity = 0;

                                if ((depth & 0x0100) == 0)
                                {
                                    intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);
                                }
                                else
                                {
                                    intensity = (byte)(depth);
                                    intensity = (byte)(255 - intensity);
                                }

                                // Write out blue byte
                                this.colorPixels[colorPixelIndex++] = intensity;

                                // Write out green byte
                                this.colorPixels[colorPixelIndex++] = intensity;

                                // Write out red byte                        
                                this.colorPixels[colorPixelIndex++] = intensity;

                            }
                        }
                        else
                        {
                            //Bad depth gets dark blue
                            
                            this.colorPixels[colorPixelIndex++] = 50;
                            this.colorPixels[colorPixelIndex++] = 0;     
                            this.colorPixels[colorPixelIndex++] = 0;
                        }
                                                
                        // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                        // If we were outputting BGRA, we would write alpha here.
                        ++colorPixelIndex;
                    }

                    //Find large contiguous regions
                    for (int i = 0; i < this.diffMap.Length; ++i)
                    {
                        this.diffMapScratch[i] = this.diffMap[i];
                    }

                    // 
                    //
                    // NOTE:  This doesn't seem to be necessary right now, will update later if it proves needed
                    //
                    //
                    ////Grow the empty area to filter out narrow regions
                    Stack<int> s = tempStack1;
                    //Stack<int> b = tempStack2;
                    int cur = -1;
                    //for (int i = 0; i < this.diffMap.Length; ++i)
                    //{
                    //    if (diffMap[i] == 0)
                    //    {
                    //        s.Push(i);
                    //        break;
                    //    }
                    //}

                    ////Do a flood fill from 0 to 2 to find the borders of the empty area,
                    //// end result is a diffMap where regions without anything are "2",
                    //// other pixels are "1".
                    //// Borders between 2 and 1 are added to the "b" stack for being expanded
                    //while (s.Count > 0)
                    //{
                    //    cur = s.Pop();
                    //    diffMap[cur] = 2;
                    //    for (int direction = 0; direction < 4; direction++)
                    //    {
                    //        int next = getNode(cur, direction);
                    //        if (next != -1)
                    //        {
                    //            switch (diffMap[next])
                    //            {
                    //                case 0:
                    //                    s.Push(next);
                    //                    break;
                    //                case 1:
                    //                    diffMap[cur] = 3;
                    //                    b.Push(next);
                    //                    break;
                    //                case 2:
                    //                default:
                    //                    break;
                    //            }
                    //        }
                    //    }
                    //}

                    //for (int i = 1; i < this.borderGrowth; i++)
                    //{
                    //    Stack<int> oldBorder;
                    //    Stack<int> newBorder;
                    //    if (i % 2 == 1)
                    //    {
                    //        oldBorder = b;
                    //        newBorder = s;
                    //    }
                    //    else
                    //    {
                    //        oldBorder = s;
                    //        newBorder = b;
                    //    }

                    //    foreach(int j in oldBorder)
                    //    {
                    //        diffMap[j] = 3;
                    //        for (int direction = 0; direction < 4; direction++)
                    //        {
                    //            int next = getNode(j, direction);
                    //            if (next != -1)
                    //            {
                    //                switch (diffMap[next])
                    //                {
                    //                    case 1:
                    //                        newBorder.Push(next);
                    //                        break;
                    //                    default:
                    //                        break;
                    //                }
                    //            }
                    //        }
                    //    }
                    //}

                    //Find large contiguous regions, remove 
                    int targetsFound = 0;
                    for (int i = 0; i < this.diffMap.Length; ++i)
                    {
                        int contiguous = 0;

                        if (diffMap[i] != 1)
                            continue;

                        s.Push(i);
                        // Flood fill 1s to 0s around pixel i, incrementing contiguous as you go
                        while (s.Count > 0)
                        {
                            contiguous++;
                            cur = s.Pop();
                            if (diffMap[cur] != 0)
                            {
                                diffMap[cur] = 0;
                                for (int direction = 0; direction < 4; direction++)
                                {
                                    int next = getNode(cur, direction);
                                    if (next != -1)
                                    {
                                        if (diffMap[next] == 1)
                                        {
                                            s.Push(next);
                                        }
                                    }
                                }
                            }
                        }

                        if (contiguous > this.targetSizeThreshold)
                        {
                            targetsFound++;
                        }
                        else
                        {
                            // If you find fewer than this.targetSizeThreshold contiguous pixels,
                            // erase this area from diffMapScratch
                            s.Push(i);

                            cur = -1;
                            while (s.Count > 0)
                            {
                                contiguous++;
                                cur = s.Pop();
                                if (diffMapScratch[cur] == 1)
                                {
                                    diffMapScratch[cur] = 0;
                                    for (int direction = 0; direction < 4; direction++)
                                    {
                                        int next = getNode(cur, direction);
                                        if (next != -1)
                                        {
                                            if (diffMapScratch[next] == 1)
                                            {
                                                s.Push(next);
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        //if finished && contiguous < targetSizeThresh
                        // fill in on diffMapScratch

                        this.diffMapScratch[i] = this.diffMap[i];
                    }

                    if (targetsFound >= 2 && systemArmed)
                    {
                        if (!alarmIsPlaying)
                        {
                            this.Player.SoundLocation = @"WARNING_INTRUDER.wav";
                            this.Player.PlayLooping();
                            alarmIsPlaying = true;
                        }
                    }
                    else
                    {
                        this.Player.Stop();
                        alarmIsPlaying = false;
                    }

                    colorPixelIndex = 0;
                    //Highlight large contiguous regions on the output
                    for (int i = 0; i < this.diffMap.Length; ++i)
                    {
                        if (this.diffMapScratch[i] == 1)
                        {
                            // blue
                            this.colorPixels[colorPixelIndex++] = 0;

                            // green
                            this.colorPixels[colorPixelIndex++] = 0;

                            // red
                            this.colorPixels[colorPixelIndex++] = 255;

                            colorPixelIndex++;
                        }
                        else
                        {
                            colorPixelIndex += 4;
                        }
                    }

                    // Write the pixel data into our bitmap
                    this.colorBitmap.WritePixels(
                    new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                    this.colorPixels,
                    this.colorBitmap.PixelWidth * sizeof(int),
                    0);
                }
            }
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// (overridden in the "Cat Detector" to also get a baseline and arm the system)
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ButtonScreenshotClick(object sender, RoutedEventArgs e)
        {
            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.ConnectDeviceFirst;
                return;
            }

            // create a png bitmap encoder which knows how to save a .png file
            BitmapEncoder encoder = new PngBitmapEncoder();

            // create frame from the writable bitmap and add to encoder
            encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap));

            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            string path = Path.Combine(myPhotos, "CatDetectorScreenshot-" + time + ".png");

            // write the new file to disk
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }

                this.statusBarText.Text = string.Format(CultureInfo.InvariantCulture, "{0} {1}", Properties.Resources.ScreenshotWriteSuccess, path);
            }
            catch (IOException)
            {
                this.statusBarText.Text = string.Format(CultureInfo.InvariantCulture, "{0} {1}", Properties.Resources.ScreenshotWriteFailed, path);
            }

            this.captureBaseline = true;
        }

        //private bool playing = false;

        private void ButtonThresholdClick(object sender, RoutedEventArgs e)
        {
            this.depthThreshold = Int32.Parse(this.depthThresholdInput.Text);
            this.targetSizeThreshold = Int32.Parse(this.sizeThresholdInput.Text);

            //if (!playing)
            //{
            //    //this.Player = new SoundPlayer(@"C:\\Users\\raymo\\OneDrive\\Dev\\CatDetector\\bin\\Release\\WARNING_INTRUDER.wav");
            //    //this.Player.SoundLocation = @"C:\\Users\\raymo\\OneDrive\\Dev\\CatDetector\\bin\\Release\\WARNING_INTRUDER.wav";
            //    this.Player.SoundLocation = @"WARNING_INTRUDER.wav";
            //    this.Player.PlayLooping();
            //}
            //else
            //{
            //    this.Player.Stop();
            //}
            //playing = !playing;
        }
    }
}