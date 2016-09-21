PCLIMP(void, PointCloud, writeRGBAbyte)(pcl::PointCloud<_PointT>::Ptr *self, THByteTensor* input)
{
  if (!self || !*self)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'self' must not be null.");
  if (!input)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'input' must not be null.");

  pcl::PointCloud<_PointT>& c = **self;

  if (c.size() * 4 > THByteTensor_nElement(input))
    PCL_THROW_EXCEPTION(TorchPclException, "Input RGBA color tensor size is too small.");

  THByteTensor *input_ = THByteTensor_newContiguous(input);
  const uint8_t *input_data = THByteTensor_data(input_);
  for (pcl::PointCloud<_PointT>::iterator i = c.begin(); i != c.end(); ++i)
  {
    _PointT& p = *i;
    p.r = *input_data++;
    p.g = *input_data++;
    p.b = *input_data++;
    p.a = *input_data++;
  }

  THByteTensor_free(input_);
}

PCLIMP(void, PointCloud, writeRGBAfloat)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor* input)
{
  if (!self || !*self)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'self' must not be null.");
  if (!input)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'input' must not be null.");

  pcl::PointCloud<_PointT>& c = **self;

  if (c.size() * 4 > THFloatTensor_nElement(input))
    PCL_THROW_EXCEPTION(TorchPclException, "Input RGBA color tensor size is too small.");

  THFloatTensor* input_ = THFloatTensor_newContiguous(input);

  const float *input_data = THFloatTensor_data(input_);
  for (pcl::PointCloud<_PointT>::iterator i = c.begin(); i != c.end(); ++i)
  {
    _PointT& p = *i;
    p.r = static_cast<uint8_t>(saturate(*input_data++) * 255);
    p.g = static_cast<uint8_t>(saturate(*input_data++) * 255);
    p.b = static_cast<uint8_t>(saturate(*input_data++) * 255);
    p.a = static_cast<uint8_t>(saturate(*input_data++) * 255);
  }

  THFloatTensor_free(input_);
}

PCLIMP(void, PointCloud, writeRGBbyte)(pcl::PointCloud<_PointT>::Ptr *self, THByteTensor* input, bool setAlpha, uint8_t alpha = 255)
{
  if (!self || !*self)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'self' must not be null.");
  if (!input)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'input' must not be null.");

  pcl::PointCloud<_PointT>& c = **self;

  if (c.size() * 3 > THByteTensor_nElement(input))
    PCL_THROW_EXCEPTION(TorchPclException, "Input RGB color tensor size is too small.");

  THByteTensor *input_ = THByteTensor_newContiguous(input);
  const uint8_t *input_data = THByteTensor_data(input_);
  for (pcl::PointCloud<_PointT>::iterator i = c.begin(); i != c.end(); ++i)
  {
    _PointT& p = *i;
    p.r = *input_data++;
    p.g = *input_data++;
    p.b = *input_data++;
    if (setAlpha)
    {
      p.a = alpha;
    }
  }

  THByteTensor_free(input_);
}

PCLIMP(void, PointCloud, writeRGBfloat)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor* input, bool setAlpha, float alpha = 1)
{
  if (!self || !*self)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'self' must not be null.");
  if (!input)
    PCL_THROW_EXCEPTION(TorchPclException, "Argument 'input' must not be null.");

  pcl::PointCloud<_PointT>& c = **self;

  if (c.size() * 3 > THFloatTensor_nElement(input))
    PCL_THROW_EXCEPTION(TorchPclException, "Input RGB color tensor size is too small.");

  THFloatTensor* input_ = THFloatTensor_newContiguous(input);

  const float *input_data = THFloatTensor_data(input_);
  for (pcl::PointCloud<_PointT>::iterator i = c.begin(); i != c.end(); ++i)
  {
    _PointT& p = *i;
    p.r = static_cast<uint8_t>(saturate(*input_data++) * 255);
    p.g = static_cast<uint8_t>(saturate(*input_data++) * 255);
    p.b = static_cast<uint8_t>(saturate(*input_data++) * 255);
    if (setAlpha)
    {
      p.a = static_cast<uint8_t>(alpha * 255);
    }
  }

  THFloatTensor_free(input_);
}
