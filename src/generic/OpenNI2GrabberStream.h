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
  {
  }

  void start()
  {
    if (!callbackRegistered)
    {
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
    boost::unique_lock<boost::mutex> lock(queueLock);

    boost::chrono::system_clock::time_point timeout = boost::chrono::system_clock::now() 
      + boost::chrono::milliseconds(timeout_milliseconds);
      
    do
    {
      // check if frames are available
      if (!queue.empty())
      {
        CloudPointPtr frame = queue.front(); 
        queue.pop_front();
        return frame;
      }

      // wait with timeout for frame to be captured
    } while(framesAvailable.wait_until(lock, timeout) != boost::cv_status::timeout);

    return CloudPointPtr();
  }
  
  pcl::io::OpenNI2Grabber& getGrabber()
  {
    return grabber;
  }
  
private:  
  int maxBacklog;
  std::deque<CloudPointPtr> queue;
  boost::mutex queueLock;
  boost::condition_variable framesAvailable;
  pcl::io::OpenNI2Grabber grabber;
  bool callbackRegistered;
  
  void grabber_callback(const CloudPointConstPtr &cloud)
  {
    boost::unique_lock<boost::mutex> lock(queueLock);
    if (maxBacklog >= 0 && queue.size() > maxBacklog)
      queue.pop_front();
    queue.push_back(cloud->makeShared());
    framesAvailable.notify_one();
  }
};
