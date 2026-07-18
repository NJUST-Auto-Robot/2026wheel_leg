#include "MachineVision/find_block.h"

BLOCK block;

void Block_Init(BLOCK* Block)
{
  for(uint8_t i = 0; i < 120; i++)
  {
    for(uint8_t j = 0; j < 188; j++)
    {
      Block->label[i][j] = 0;
    }
  }
}

void Find_Block(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block)
{
  uint8_t nx = 0;
  uint8_t ny = 0;
  
  Block->block_num = 0;
  Block->front = 0;
  Block->rear = 0;
  
  for(uint8_t y = 0; y < height; y++)
  {
    for(uint8_t x = 0; x < width; x++)
    {
      if(image[y * width + x] == 255 && Block->label[y][x] == 0)
      {
        Block->block_num += 1;
        Block->queue_x[Block->rear] = x;
        Block->queue_y[Block->rear] = y;
        Block->rear += 1;
        Block->label[y][x] = Block->block_num;
        
        while(Block->front < Block->rear)
        {
          nx = Block->queue_x[Block->front];
          ny = Block->queue_y[Block->front];
          
          if(nx - 1 >= 0 && Block->label[ny][(nx - 1)] == 0 && image[ny * width + (nx - 1)] == 255)
          {
             Block->queue_x[Block->rear] = nx - 1;
             Block->queue_y[Block->rear] = ny;
             Block->rear += 1;
             Block->label[ny][(nx - 1)] = Block->block_num;
          }
          
          if(ny - 1 >= 0 && Block->label[(ny - 1)][nx] == 0 && image[(ny - 1) * width + nx] == 255)
          {
             Block->queue_x[Block->rear] = nx;
             Block->queue_y[Block->rear] = ny - 1;
             Block->rear += 1;
             Block->label[(ny - 1)][nx] = Block->block_num;
          }
          
          if(nx + 1 < width && Block->label[ny][(nx + 1)] == 0 && image[ny * width + (nx + 1)] == 255)
          {
             Block->queue_x[Block->rear] = nx + 1;
             Block->queue_y[Block->rear] = ny;
             Block->rear += 1;
             Block->label[ny][(nx + 1)] = Block->block_num;
          }
          if(ny + 1 < height && Block->label[(ny + 1)][nx] == 0 && image[(ny -+ 1) * width + nx] == 255)
          {
             Block->queue_x[Block->rear] = nx;
             Block->queue_y[Block->rear] = ny + 1;
             Block->rear += 1;
             Block->label[(ny + 1)][nx] = Block->block_num;
          }
          Block->front += 1;
        }
        Block->front = 0;
        Block->rear = 0;
      }
      else if(image[y * width + x] == 255 && Block->label[y][x] != 0)
      {
        Block->label[y][x] = 0;
      }
    }
  }
}

void Find_Block_Pro(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block, uint8_t max_threshold, uint8_t min_threshold)
{
  uint8_t nx = 0;
  uint8_t ny = 0;
  
  Block->block_num = 0;
  Block->front = 0;
  Block->rear = 0;
  
  for(uint8_t y = 0; y < height; y++)
  {
    for(uint8_t x = 0; x < width; x++)
    {
      if(image[y * width + x] >= min_threshold && image[y * width + x] <= max_threshold && Block->label[y][x] == 0)
      {
        Block->block_num += 1;
        Block->queue_x[Block->rear] = x;
        Block->queue_y[Block->rear] = y;
        Block->rear += 1;
        Block->label[y][x] = Block->block_num;
        //为初始中心点、边界点赋初值
        Block->cx[Block->block_num] = 0;
        Block->cy[Block->block_num] = 0;
        Block->max_x[Block->block_num] = x;
        Block->max_y[Block->block_num] = y;
        Block->min_x[Block->block_num] = x;
        Block->min_y[Block->block_num] = y;
        //队列式洪水填充法
        while(Block->front < Block->rear)
        {
          nx = Block->queue_x[Block->front];
          ny = Block->queue_y[Block->front];
          //色块中心点累加
          Block->cx[Block->block_num] += nx;
          Block->cy[Block->block_num] += ny;
          //色块边界点取舍
          if(Block->max_x[Block->block_num] < nx)
            Block->max_x[Block->block_num] = nx;
          
          if(Block->max_y[Block->block_num] < ny)
            Block->max_y[Block->block_num] = ny;
          
          if(Block->min_x[Block->block_num] > nx)
            Block->min_x[Block->block_num] = nx;
          
          if(Block->min_y[Block->block_num] > ny)
            Block->min_y[Block->block_num] = ny;
          //四邻域查找目标像素点
          if(nx - 1 >= 0 && Block->label[ny][(nx - 1)] == 0 && image[ny * width + (nx - 1)] >= min_threshold && image[ny * width + (nx - 1)] <= max_threshold)
          {
             Block->queue_x[Block->rear] = nx - 1;
             Block->queue_y[Block->rear] = ny;
             Block->rear += 1;
             Block->label[ny][(nx - 1)] = Block->block_num;
          }
          
          if(ny - 1 >= 0 && Block->label[(ny - 1)][nx] == 0 && image[(ny - 1) * width + nx] >= min_threshold && image[(ny - 1) * width + nx] <= max_threshold)
          {
             Block->queue_x[Block->rear] = nx;
             Block->queue_y[Block->rear] = ny - 1;
             Block->rear += 1;
             Block->label[(ny - 1)][nx] = Block->block_num;
          }
          
          if(nx + 1 < width && Block->label[ny][(nx + 1)] == 0 && image[ny * width + (nx + 1)] == min_threshold && image[ny * width + (nx + 1)] <= max_threshold)
          {
             Block->queue_x[Block->rear] = nx + 1;
             Block->queue_y[Block->rear] = ny;
             Block->rear += 1;
             Block->label[ny][(nx + 1)] = Block->block_num;
          }
          if(ny + 1 < height && Block->label[(ny + 1)][nx] == 0 && image[(ny + 1) * width + nx] == min_threshold && image[(ny + 1) * width + nx] <= max_threshold)
          {
             Block->queue_x[Block->rear] = nx;
             Block->queue_y[Block->rear] = ny + 1;
             Block->rear += 1;
             Block->label[(ny + 1)][nx] = Block->block_num;
          }
          Block->front += 1;
        }
        //色块面积即该色块内像素个数即队列的队尾值
        Block->pixel[Block->block_num] = Block->rear;
        //色块中心点最后的求平均计算
        Block->cx[Block->block_num] /= Block->rear;
        Block->cy[Block->block_num] /= Block->rear;
        //队列清零
        Block->front = 0;
        Block->rear = 0;
      }
      else if(image[y * width + x] >= min_threshold && image[y * width + x] <= max_threshold && Block->label[y][x] != 0)
      {
        Block->label[y][x] = 0;
      }
    }
  }
}

void Find_Block_Pro_Max(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block, uint8_t max_threshold, uint8_t min_threshold, uint8_t roi_x, uint8_t roi_y, uint8_t roi_w, uint8_t roi_h)
{
  //变量定义以及初始化
  uint8_t nx = 0;
  uint8_t ny = 0;
  
  Block->block_num = 0;
  Block->front = 0;
  Block->rear = 0;
  //开启洪水填充找色块
  for(uint8_t y = roi_y; y < roi_y + roi_h; y++)
  {
    for(uint8_t x = roi_x; x < roi_x + roi_w; x++)
    {
      if(image[y * width + x] >= min_threshold && image[y * width + x] <= max_threshold && Block->label[y][x] == 0)
      {
        
        if(Block->block_num > 99)
          break;
        Block->block_num += 1;
        Block->queue_x[Block->rear] = x;
        Block->queue_y[Block->rear] = y;
        Block->rear += 1;
        Block->label[y][x] = Block->block_num;
        //为初始中心点、边界点赋初值
        Block->cx[Block->block_num] = 0;
        Block->cy[Block->block_num] = 0;
        Block->max_x[Block->block_num] = x;
        Block->max_y[Block->block_num] = y;
        Block->min_x[Block->block_num] = x;
        Block->min_y[Block->block_num] = y;
        //队列式洪水填充法
        while(Block->front < Block->rear)
        {
          nx = Block->queue_x[Block->front];
          ny = Block->queue_y[Block->front];
          //色块中心点累加
          Block->cx[Block->block_num] += nx;
          Block->cy[Block->block_num] += ny;
          //色块边界点取舍
          if(Block->max_x[Block->block_num] < nx)
            Block->max_x[Block->block_num] = nx;
          
          if(Block->max_y[Block->block_num] < ny)
            Block->max_y[Block->block_num] = ny;
          
          if(Block->min_x[Block->block_num] > nx)
            Block->min_x[Block->block_num] = nx;
          
          if(Block->min_y[Block->block_num] > ny)
            Block->min_y[Block->block_num] = ny;
          //四邻域查找目标像素点
          if(nx - 1 >= roi_x && Block->label[ny][(nx - 1)] == 0 && image[ny * width + (nx - 1)] >= min_threshold && image[ny * width + (nx - 1)] <= max_threshold)
          {
             Block->queue_x[Block->rear] = nx - 1;
             Block->queue_y[Block->rear] = ny;
             Block->rear += 1;
             Block->label[ny][(nx - 1)] = Block->block_num;
          }
          
          if(ny - 1 >= roi_y && Block->label[(ny - 1)][nx] == 0 && image[(ny - 1) * width + nx] >= min_threshold && image[(ny - 1) * width + nx] <= max_threshold)
          {
             Block->queue_x[Block->rear] = nx;
             Block->queue_y[Block->rear] = ny - 1;
             Block->rear += 1;
             Block->label[(ny - 1)][nx] = Block->block_num;
          }
          
          if(nx + 1 < roi_x + roi_w && Block->label[ny][(nx + 1)] == 0 && image[ny * width + (nx + 1)] == min_threshold && image[ny * width + (nx + 1)] <= max_threshold)
          {
             Block->queue_x[Block->rear] = nx + 1;
             Block->queue_y[Block->rear] = ny;
             Block->rear += 1;
             Block->label[ny][(nx + 1)] = Block->block_num;
          }
          if(ny + 1 < roi_y + roi_h && Block->label[(ny + 1)][nx] == 0 && image[(ny + 1) * width + nx] == min_threshold && image[(ny + 1) * width + nx] <= max_threshold)
          {
             Block->queue_x[Block->rear] = nx;
             Block->queue_y[Block->rear] = ny + 1;
             Block->rear += 1;
             Block->label[(ny + 1)][nx] = Block->block_num;
          }
          Block->front += 1;
        }
        //色块面积即该色块内像素个数即队列的队尾值
        Block->pixel[Block->block_num] = Block->rear;
        //色块中心点最后的求平均计算
        Block->cx[Block->block_num] /= Block->rear;
        Block->cy[Block->block_num] /= Block->rear;
        //队列清零
        Block->front = 0;
        Block->rear = 0;
      }
      //标签清零，为了下一帧图像的再一次查找
      else if(image[y * width + x] >= min_threshold && image[y * width + x] <= max_threshold && Block->label[y][x] != 0)
      {
        Block->label[y][x] = 0;
      }
    }
  }
  
  //将找到的所有色块合并成大色块
  Block->cx[0] = 0;
  Block->cy[0] = 0;
  Block->max_x[0] = Block->max_x[Block->block_num];
  Block->max_y[0] = Block->max_y[Block->block_num];
  Block->min_x[0] = Block->min_x[Block->block_num];
  Block->min_y[0] = Block->min_y[Block->block_num];
  
  for(uint8_t num = 1; num <= Block->block_num; num++)
  {
      if(Block->max_x[0] < Block->max_x[num])
        Block->max_x[0] = Block->max_x[num];
      
      if(Block->max_y[0] < Block->max_y[num])
        Block->max_y[0] = Block->max_y[num];
      
      if(Block->min_x[0] > Block->min_x[num])
        Block->min_x[0] = Block->min_x[num];
      
      if(Block->min_y[0] > Block->min_y[num])
        Block->min_y[0] = Block->min_y[num];
      
      Block->cx[0] += Block->cx[num] / Block->block_num;
      Block->cy[0] += Block->cy[num] / Block->block_num;
  }
}
//画出Block结构体中全部色块边框
void Draw_Block(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block)
{
  if(Block->block_num != 0)
  {
    for(uint8_t num = 1; num <= Block->block_num; num++)
    {
      for(uint8_t y = Block->min_y[num]; y <= Block->max_y[num]; y++)
      {
        image[y * width + Block->max_x[num]] = 255;
        image[y * width + Block->min_x[num]] = 255;
      }
      for(uint8_t x = Block->min_x[num]; x <= Block->max_x[num]; x++)
      {
        image[Block->max_y[num] * width + x] = 255;
        image[Block->min_y[num] * width + x] = 255;
      }
    }
  }
}
//画出Block结构体中所有色块合并在一起的大色块的边框
void Draw_Max_Block(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block)
{
  uint8_t which_one_have_the_most_pixels = 0;
  uint32_t max_pixels = 0;
  if(Block->block_num != 0)
  {
    for(uint8_t num = 1; num < Block->block_num; num++)
    {
      if(max_pixels < Block->pixel[num])
      {
        which_one_have_the_most_pixels = num;
        max_pixels = Block->pixel[num];
      }
    }

    for(uint8_t y = Block->min_y[which_one_have_the_most_pixels]; y <= Block->max_y[which_one_have_the_most_pixels]; y++)
    {
      image[y * width + Block->max_x[which_one_have_the_most_pixels]] = 255;
      image[y * width + Block->min_x[which_one_have_the_most_pixels]] = 255;
    }
    for(uint8_t x = Block->min_x[which_one_have_the_most_pixels]; x <= Block->max_x[which_one_have_the_most_pixels]; x++)
    {
      image[Block->max_y[which_one_have_the_most_pixels] * width + x] = 255;
      image[Block->min_y[which_one_have_the_most_pixels] * width + x] = 255;
    }
  }
}

void Draw_Merge_Block(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block)
{
  if(Block->block_num != 0)
  {
    for(uint8_t y = Block->min_y[0]; y <= Block->max_y[0]; y++)
    {
      image[y * width + Block->max_x[0]] = 255;
      image[y * width + Block->min_x[0]] = 255;
    }
    for(uint8_t x = Block->min_x[0]; x <= Block->max_x[0]; x++)
    {
      image[Block->max_y[0] * width + x] = 255;
      image[Block->min_y[0] * width + x] = 255;
    }
  }
}