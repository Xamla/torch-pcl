#include <pcl/io/openni2_grabber.h>

// Allow single threaded access to open NI point cloud stream from main LUA thread.
// Up to maxBacklog frames are buffered.
template<typename PointT>
class OpenNI2GrabberStream
{
public:
  typedef typename pcl::PointCloud<PointT>::Ptr CloudPointPtr;
  typedef typename pcl::PointCloud<PointT>::ConstPtr CloudPointConstPtr;

  OpenNI2GrabberStream(const std::string& device_id, int max_backlog = 30)
    : maxBacklog(max_backlog)
    , grabber(device_id)
    , callbackRegistered(false)
    , grabImages(false)
    , grabIRImages(false)
  {
  }

  void setGrabImages(bool flag)
  {
    grabImages = flag;
  }
  
  void setGrabIRImages(bool flag)
  {
    grabIRImages = flag;
  }

  void start()
  {
    if (!callbackRegistered)
    {
      if (grabIRImages)
      {
        boost::function<void (const boost::shared_ptr<pcl::io::IRImage>&)> cb =
          boost::bind (&OpenNI2GrabberStream::grabber_ir_image_callback, this, _1);
        grabber.registerCallback(cb);
      }
      
      if (grabImages)
      {
        boost::function<void (const boost::shared_ptr<pcl::io::Image>&)> cb =
          boost::bind (&OpenNI2GrabberStream::grabber_image_callback, this, _1);
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
    queue.clear();
  }
  
  size_t getCount()
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    return queue.size();
  }
  
  CloudPointPtr read(int timeout_milliseconds)
  {
    return readFromQueue(queue, framesAvailable, timeout_milliseconds);
  }
  
  pcl::io::Image::Ptr readImage(int timeout_milliseconds)
  {
    return readFromQueue(image_queue, imagesAvailable, timeout_milliseconds);
  }
  
  pcl::io::IRImage::Ptr readIRImage(int timeout_milliseconds)
  {
    return readFromQueue(irimage_queue, irimagesAvailable, timeout_milliseconds);
  }
  
  pcl::io::OpenNI2Grabber& getGrabber()
  {
    return grabber;
  }
  
private:
  int maxBacklog;
  std::deque<CloudPointPtr> queue;
  std::deque<pcl::io::Image::Ptr> image_queue;
  std::deque<pcl::io::IRImage::Ptr> irimage_queue;
  
  boost::mutex queueLock;
  boost::condition_variable framesAvailable;
  boost::condition_variable imagesAvailable;
  boost::condition_variable irimagesAvailable;
  
  pcl::io::OpenNI2Grabber grabber;
  bool callbackRegistered;
  bool grabImages;
  bool grabIRImages;
  
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
    if (maxBacklog >= 0 && queue.size() > maxBacklog)
      queue.pop_front();
    queue.push_back(cloud->makeShared());
    framesAvailable.notify_one();
  }
  
  void grabber_image_callback(const boost::shared_ptr<pcl::io::Image> &image)
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    if (maxBacklog >= 0 && image_queue.size() > maxBacklog)
      image_queue.pop_front();
    image_queue.push_back(image);
    imagesAvailable.notify_one();
  }
  
  void grabber_ir_image_callback(const boost::shared_ptr<pcl::io::IRImage> &irimage)
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    if (maxBacklog >= 0 && irimage_queue.size() > maxBacklog)
      irimage_queue.pop_front();
    irimage_queue.push_back(irimage);
    irimagesAvailable.notify_one();
  }
};
