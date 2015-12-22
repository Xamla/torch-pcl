PCLIMP(void, PointCloud, readRGBAbyte)(pcl::PointCloud<_PointT>::Ptr *self, THByteTensor* output)
{
  if (!self || !*self)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'self' must not be null.");
  if (!output)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'output' must not be null.");
    
  const pcl::PointCloud<_PointT>& c = **self;

  THByteTensor_resize3d(output, c.height, c.width, 4);
  
  THByteTensor *output_ = THByteTensor_newContiguous(output);
  unsigned char *output_data = THByteTensor_data(output_);
  for (pcl::PointCloud<_PointT>::const_iterator i = c.begin(); i != c.end(); ++i)
  {
    const _PointT& p = *i;
    *output_data++ = p.r;
    *output_data++ = p.g;
    *output_data++ = p.b;
    *output_data++ = p.a;
  }
  
  THByteTensor_freeCopyTo(output_, output);
}

PCLIMP(void, PointCloud, readRGBAfloat)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor* output)
{
  if (!self || !*self)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'self' must not be null.");
  if (!output)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'output' must not be null.");
    
  const pcl::PointCloud<_PointT>& c = **self;
  
  THFloatTensor_resize3d(output, c.height, c.width, 4);
  
  THFloatTensor* output_ = THFloatTensor_newContiguous(output);

  float *output_data = THFloatTensor_data(output_);
  for (pcl::PointCloud<_PointT>::const_iterator i = c.begin(); i != c.end(); ++i)
  {
    const _PointT& p = *i;
    *output_data++ = p.r / 255.0f;
    *output_data++ = p.g / 255.0f;
    *output_data++ = p.b / 255.0f;
    *output_data++ = p.a / 255.0f;
  }
  
  THFloatTensor_freeCopyTo(output_, output);
}
