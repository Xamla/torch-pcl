#include <pcl/io/openni2_grabber.h>

// Allow single threaded access to open NI point cloud stream from main LUA thread.
// Up to maxBacklog frames are buffered.
template<typename PointT>
class OpenNI2GrabberStream
{
public:
  typedef typename pcl::PointCloud<PointT>::Ptr CloudPointPtr;
  typedef typename pcl::PointCloud<PointT>::ConstPtr CloudPointConstPtr;

  OpenNI2GrabberStream(const std::string& device_id, int max_backlog = 30, bool grab_RGB = false, bool grab_Depth = false, bool grab_IR = false)
    : maxBacklog(max_backlog)
    , grabber(device_id)
    , callbackRegistered(false)
    , grab_RGB(grab_RGB)
    , grab_Depth(grab_Depth)
    , grab_IR(grab_IR)
  {
  }

  void start()
  {
    if (!callbackRegistered)
    {
      if (grab_RGB)
      {
        boost::function<void (const boost::shared_ptr<pcl::io::Image>&)> cb =
          boost::bind (&OpenNI2GrabberStream::grabber_rgb_image_callback, this, _1);
        grabber.registerCallback(cb);
      }
      
      if (grab_Depth)
      {
        boost::function<void (const boost::shared_ptr<pcl::io::DepthImage>&)> cb =
          boost::bind (&OpenNI2GrabberStream::grabber_depth_image_callback, this, _1);
        grabber.registerCallback(cb);
      }
      
      if (grab_IR)
      {
        boost::function<void (const boost::shared_ptr<pcl::io::IRImage>&)> cb =
          boost::bind (&OpenNI2GrabberStream::grabber_ir_image_callback, this, _1);
        grabber.registerCallback(cb);
      }

      boost::function<void (const CloudPointConstPtr&)> cb =
        boost::bind (&OpenNI2GrabberStream::grabber_callback, this, _1);
      grabber.registerCallback(cb);
      callbackRegistered = true;
    }
    
    boost::unique_lock<boost::mutex> lock(queueLock);
    if (!grabber.isRunning())
      grabber.start();
  }
  
  void stop()
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    grabber.stop();
  }
  
  bool isRunning()
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    return grabber.isRunning();
  }

  void clear()
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    cloud_Queue.clear();
    rgb_Queue.clear();
    depth_Queue.clear();
    ir_Queue.clear();
  }
  
  size_t getCount()
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    return cloud_Queue.size();
  }
  
  CloudPointPtr read(int timeout_milliseconds)
  {
    return readFromQueue(cloud_Queue, framesAvailable, timeout_milliseconds);
  }
  
  pcl::io::Image::Ptr readRGBImage(int timeout_milliseconds)
  {
    return readFromQueue(rgb_Queue, rgb_ImagesAvailable, timeout_milliseconds);
  }
  
  pcl::io::DepthImage::Ptr readDepthImage(int timeout_milliseconds)
  {
    return readFromQueue(depth_Queue, depth_ImagesAvailable, timeout_milliseconds);
  }
  
  pcl::io::IRImage::Ptr readIRImage(int timeout_milliseconds)
  {
    return readFromQueue(ir_Queue, ir_ImagesAvailable, timeout_milliseconds);
  }
  
  pcl::io::OpenNI2Grabber& getGrabber()
  {
    return grabber;
  }
  
private:
  int maxBacklog;
  std::deque<CloudPointPtr> cloud_Queue;
  std::deque<pcl::io::Image::Ptr> rgb_Queue;
  std::deque<pcl::io::DepthImage::Ptr> depth_Queue;
  std::deque<pcl::io::IRImage::Ptr> ir_Queue;
  
  boost::mutex queueLock;
  boost::condition_variable framesAvailable;
  boost::condition_variable rgb_ImagesAvailable;
  boost::condition_variable ir_ImagesAvailable;
  boost::condition_variable depth_ImagesAvailable;
  
  pcl::io::OpenNI2Grabber grabber;
  bool callbackRegistered;
  bool grab_RGB;
  bool grab_Depth;
  bool grab_IR;
  
  template<typename T>
  T readFromQueue(std::deque<T>& queue, boost::condition_variable& condition, int timeout_milliseconds)
  {
    boost::unique_lock<boost::mutex> lock(queueLock);

    boost::chrono::system_clock::time_point timeout = boost::chrono::system_clock::now() 
      + boost::chrono::milliseconds(timeout_milliseconds);
      
    do
    {
      // check if frames are available
      if (!queue.empty())
      {
        T frame = queue.front(); 
        queue.pop_front();
        return frame;
      }

      // wait with timeout for frame to be captured
    } while(condition.wait_until(lock, timeout) != boost::cv_status::timeout);

    return T();
  }
  
  void grabber_callback(const CloudPointConstPtr &cloud)
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    if (maxBacklog >= 0 && cloud_Queue.size() > maxBacklog)
      cloud_Queue.pop_front();
    cloud_Queue.push_back(cloud->makeShared());
    framesAvailable.notify_one();
  }
  
  void grabber_rgb_image_callback(const boost::shared_ptr<pcl::io::Image> &image)
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    if (maxBacklog >= 0 && rgb_Queue.size() > maxBacklog)
      rgb_Queue.pop_front();
    rgb_Queue.push_back(image);
    rgb_ImagesAvailable.notify_one();
  }
  
  void grabber_depth_image_callback(const boost::shared_ptr<pcl::io::DepthImage> &image)
  {
      boost::unique_lock<boost::mutex> lock(queueLock);
    if (maxBacklog >= 0 && depth_Queue.size() > maxBacklog)
      depth_Queue.pop_front();
    depth_Queue.push_back(image);
    depth_ImagesAvailable.notify_one();
  }
  
  void grabber_ir_image_callback(const boost::shared_ptr<pcl::io::IRImage> &irimage)
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    if (maxBacklog >= 0 && ir_Queue.size() > maxBacklog)
      ir_Queue.pop_front();
    ir_Queue.push_back(irimage);
    ir_ImagesAvailable.notify_one();
  }
};
