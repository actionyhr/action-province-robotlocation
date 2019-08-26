/*

- @Author: mikey.zhaopeng 

- @Date: 2019-08-01 09:11:59 

- @Last Modified by: mikey.zhaopeng

- @Last Modified time: 2019-08-01 15:34:02
  */
  #include "robot_locator.h"
  std::ofstream outfile("test.txt");
  RobotLocator::RobotLocator()
  {
  srcImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  dstImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  bgImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  allBallImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelB = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelG = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelR = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);

  channelH = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelS = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);

  channelL = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelA = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);

  LABImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);
  HSVImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);

  depthImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_16UC1);

}

void RobotLocator::init(ActD435& d435)
{

    cout << "Initializing locator..." << endl;

    //-- Set input device
    thisD435 = &d435;

    thisD435->getCameraParam(color_intrin, depth_intrin, depth2color_extrin, color2depth_extrin, m_data);		
    //-- Drop several frames for stable point cloud
    for (int i = 0; i < 3; i++)
    {
        thisD435->update();
    }
    cout << "Locator init done..." << endl;

    }
    void RobotLocator::updateImage(void)
    {   
        thisD435->update();
        srcImage = thisD435->getSrcImage().clone();
        depthImage = thisD435->getDepthImage().clone();
        m_data = thisD435->getData();
        depthData = thisD435->data;
    //thisTagDetector->apriltagDetect(srcImage);


}
void RobotLocator::sign_color(cv::Mat &color)
{
   cv::Mat color_hsv;
   cv::cvtColor(color,color_hsv,cv::COLOR_BGR2HSV); 
   for(int i=0;i<230;i++)
    {
        auto p=color.ptr(i);
        for(int j=0;j<640;j++)
        {
          p[j*3]=255;
          p[j*3+1]=255;
          p[j*3+2]=255;
        }
    }
  for(int i=230;i<480;i++)
  {
     uchar* p=color.ptr(i);
     const uchar* hsv_data=color_hsv.ptr(i);
      for(int j=0;j<640;j++)
      {
         /* if(hsv_data[j*3]>95&&hsv_data[j*3]<115&&hsv_data[j*3+1]>190&&hsv_data[j*3+2]>20&&hsv_data[j*3+2]<150)
          {
                    p[j*3]=255;
                    p[j*3+1]=0;
                    p[j*3+2]=0;
          }
          else if(hsv_data[j*3]>60&&hsv_data[j*3]<88&&hsv_data[j*3+1]>100&&hsv_data[j*3+2]>20)
          {
                    p[j*3]=0;
                    p[j*3+1]=255;
                    p[j*3+2]=0;
          }
          else if(hsv_data[j*3]>0&&hsv_data[j*3]<5&&hsv_data[j*3+1]>150&&hsv_data[j*3+2]>10)
          {
                    p[j*3]=0;
                    p[j*3+1]=0;
                    p[j*3+2]=255;
          }
           else if(hsv_data[j*3]>160&&hsv_data[j*3]<180&&hsv_data[j*3+1]>200&&hsv_data[j*3+2]>10)
          {
                    p[j*3]=0;
                    p[j*3+1]=0;
                    p[j*3+2]=255;
          }*/
           if(hsv_data[j*3]>5&&hsv_data[j*3]<35&&hsv_data[j*3+1]>100)
          {
                    
                    p[j*3]=0;
                    p[j*3+1]=0;
                    p[j*3+2]=0;
          }
          else
          {
                    p[j*3]=255;
                    p[j*3+1]=255;
                    p[j*3+2]=255;
          }
          

      }
  }
}
void RobotLocator::findMaxAndIndex(int x[],int length,int& max,int& index)
{
  max=0;
  index=0;
  for(int i=0;i<length;i++)
  {
    max=max>x[i]?max:x[i];
  }
  for(int i=0;i<length;i++)
  {
    if(max==x[i])
    {
      index=i;
      break;
    }
  }
}
void RobotLocator::imagePrecess(void)
{
    afterLine = srcImage.clone();
    SigedColorImage=srcImage.clone();
    sign_color(SigedColorImage);
}
void RobotLocator::findBoundary(void)
{
    cv::TickMeter tk;
    tk.start();
	  imagePrecess();
	  std::vector<HoughLine>lines;
	  cv::Point2f corner;
    getBoundaryPoint(SigedColorImage);
    findLine(&lines);   
    float point[3];
    float pixel[2];
    if(lines.size()>0)
    {   
        threeDDoundary();
        for(int i=0; i<lines.size(); i++)
        {    
            cv::line(afterLine, cv::Point(lines[i].x1, lines[i].y1),cv:: Point(lines[i].x2, lines[i].y2), cv::Scalar(255,0,0), 3);
        } 
    }
    else
    {
        return;
    }
    tk.stop();

}
void RobotLocator::threeDDoundary(void)
{

    float point[3];
    float pixel[2];
    cv::Point2d xzPoint;
    vector<HoughLine> linePar;
    lineStatus=0;
    for (size_t i = 0; i < firstVector.size(); i++)
    {
        
        vector<cv::Point2f> boundaryPoint;
        HoughLine twoLines;
        cv::Vec4f fitLine;
        for (size_t j = 0; j < firstVector[i].size(); j++)
        {
            pixel[0] = firstVector[i][j].x;
            pixel[1] = firstVector[i][j].y;
            getPointFromPixel(pixel,point);   
            xzPoint.x = point[0];
            xzPoint.y = point[2];
            boundaryPoint.push_back(xzPoint);            
        }
        if(boundaryPoint.size()>5)
            cv::fitLine(boundaryPoint,fitLine,cv::DIST_L2,0,0.01,0.01);
        else 
        {
            cout<<endl;
            return;
        }
            
        float a = fitLine[1]/fitLine[0];
        float b = fitLine[3] - a * fitLine[2];  

        twoLines.lineSlop = a;
        twoLines.intercept = b;
        twoLines.distance = fabs(b)/sqrt(pow(a,2)+1);
        linePar.push_back(twoLines);
    }
    cout<<endl;
    outfile<<"end"<<std::endl;
    cv::Point2f corner;
    float angle = 0.0f;
    if(linePar.size()==2)
    {
        findCorner(linePar,&corner);
        angle = atanf(corner.y/corner.x)/CV_PI*180.f;
       cout<<" a1 "<<linePar[0].lineSlop<<" dis1 "<<linePar[0].distance
        <<" a2 "<<linePar[1].lineSlop<<" dis2 "<<linePar[1].distance;

        cornerAngle = angle;
        leftDistance = linePar[1].distance;
        rightDistance = linePar[0].distance;
        lineStatus=3;
    }
    else if(linePar.size()==1)
    {
        angle = atanf(-1/linePar[0].lineSlop)/CV_PI*180.f;
        cout<<" a1 "<<linePar[0].lineSlop<<" dis1 "<<linePar[0].distance;

        cornerAngle = angle;
        if(linePar[0].lineSlop < 0)
        {
            leftDistance = linePar[0].distance;
            lineStatus=2;
        }   
        else
        {
            rightDistance = linePar[0].distance;
            lineStatus=1;
        }
        
    }
    cout<<" angle: "<<angle<<"status : "<<lineStatus;
}
void RobotLocator::getPointFromPixel(float colorPixel[2],float point[3])
{
    float depthPixel[2];
    rs2_project_color_pixel_to_depth_pixel(depthPixel,depthData,1.0f,0.5,5,&depth_intrin,&color_intrin,
                                            &color2depth_extrin,&depth2color_extrin,colorPixel);
    
    int raw = depthPixel[1];
    int clo = depthPixel[0];
    float depth = depthData[raw * depth_intrin.width + clo];
    rs2_deproject_pixel_to_point(point , &depth_intrin, depthPixel ,depth);
}
void RobotLocator::findCorner(std::vector<HoughLine> &lines, cv::Point2f *corner)
{
    //cv::Point corner;
    corner->x = -(lines[0].intercept-lines[1].intercept)/(lines[0].lineSlop-lines[1].lineSlop);
    corner->y = lines[0].lineSlop * corner->x + lines[0].intercept - 15;
}
void RobotLocator::getBoundaryPoint(cv::Mat src)
{


  point.clear();
  int flag=0;    
  int above[640]={ 0 };
  int below[640]={ 0 };
  int mode=0;
  int num=0;
  uchar* p=src.ptr(231);
  for(int cols=0;cols<640;cols++)
  {
    flag=0;
    for(int i=0;i<10;i++)
    {
      uchar blue=p[(cols+i)*3];
      uchar green=p[(cols+i)*3+1];
      uchar red=p[(cols+i)*3+2];
      if(!(blue==0&&green==0&&red==0))
      {
        flag=1;
        break;
      }
    }
    if(flag==0)
    {
        num++;
    }
  }
  if(num>100)
  mode=1;
  for(int cols=0;cols<640;cols++)
  {   
    for(int rows=470;rows>240;rows--)
    {
        flag=0;
        for(int i=0;i<10;i++)
        {
          uchar* ptr=src.ptr(rows-i);
          uchar b=ptr[cols*3];
          uchar g=ptr[cols*3+1];
          uchar r=ptr[cols*3+2];
          if(!(b==0&&g==0&&r==0))
          {
            flag=1;
            break;
          }
        }
        if(flag==0)
        {
          if(rows>=475)
          {
            below[cols]=0;
            break;
          }      
          else 
          { 
            below[cols]=rows;
            break;
          }
        }
    }
  }
   for(int cols=0;cols<640;cols++)
  {   
    for(int rows=470;rows>240;rows--)
    {
       flag=0;
        for(int i=1;i<10;i++)
        {
          uchar* ptr1=src.ptr(rows-i);
          uchar b1=ptr1[cols*3];
          uchar g1=ptr1[cols*3+1];
          uchar r1=ptr1[cols*3+2];
          uchar* ptr2=src.ptr(rows+i);
          uchar b2=ptr2[cols*3];
          uchar g2=ptr2[cols*3+1];
          uchar r2=ptr2[cols*3+2];
          if(!(b1==255&&g1==255&&r1==255))
          {
            flag=1;
            break;
          }
          if(!(b2==0&&g2==0&&r2==0))
          {
            flag=1;
            break;
          }
        }
        if(flag==0)
        {
            above[cols]=rows;
            break;
        }

    }
  }
  for(int i=0;i<640;i++)
  {
    if((abs(above[i]-below[i])<100&&abs(above[i]-below[i])>10&&above[i]!=0&&below[i]!=0)||mode==1)
    {
       if(i<=10)
         point.push_back(cv::Point2d(i,below[i]));
       else if(abs(below[i]-below[i-1])<3&&abs(below[i]-below[i-2])<4&&abs(below[i]-below[i-3])<5)
         point.push_back(cv::Point2d(i,below[i]));
       else if(below[i-1]==0)
         point.push_back(cv::Point2d(i,below[i]));
         
    }
  }

}
void RobotLocator::findLine(std::vector<HoughLine>*lines)
{
    firstVector.clear();
    HoughLine twoLines;
    cv::Mat showIMage = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,cv::Scalar(255,255,255));
    cv::Mat drawLine=cv::Mat::zeros(cv::Size(640,480),CV_8UC1);
    for(int i=0;i<point.size();i++)
    {   
      auto ptr=drawLine.ptr((int)point.at(i).y);
      ptr[(int)point.at(i).x]=255;   
    }
    vector<cv::Vec2f> line;
    vector<cv::Vec2f> lines_up; 
    vector<cv::Vec2f> lines_down;  
    vector<cv::Point2d> pointVector;
    vector<cv::Point2d> pointVector2;
    double twoLine_K[2]={ 0 };
    double twoLine_b[2]={ 0 };
    double isused[2]={ 0 };
    cv::HoughLines(drawLine,line,1,CV_PI/180,50,0,0);
    int mostFittedLineUpIndex=0;
    int mostFittedLineDownIndex=0;
    int MaxFittedPointNumOnLineUp=0;
    int MaxFittedPointNumOnLineDown=0;
    //将霍夫线变换找到的直线分为两类，一个是上升趋势的一个是下降趋势的
    if(line.size()>0)
    {
        for(int i=0;i<line.size();i++)
        {
            float rho=line[i][0],theta=line[i][1];
            double k=-(cos(theta)/sin(theta));
            double b=rho/sin(theta);
            if(k>=0)
            lines_up.push_back(line.at(i));
            else
            lines_down.push_back(line.at(i));
        }
    }
  
    //找出每条直线中内点(距离直线垂直距离小于5)的个数，同时根据内点的个数选出最优直线解
    int fittedPointNumUP[1000]={ 0 };
    int fittedPointNumDown[1000]={ 0 };
    if(lines_up.size()>0)
    {
        for(int i=0;i<lines_up.size();i++)
        {
            float rho=lines_up[i][0],theta=lines_up[i][1];
            double k=-(cos(theta)/sin(theta));
            double b=rho/sin(theta);
            for(int j=0;j<point.size();j++)
            {
                double distance=fabs(point.at(j).y-k*point.at(j).x-b)/sqrt(k*k+1);
                if(distance<5)
                fittedPointNumUP[i]++;
            }
        }
        findMaxAndIndex(fittedPointNumUP,lines_up.size(),MaxFittedPointNumOnLineUp,mostFittedLineUpIndex);
    }
    if(lines_down.size()>0)
    {
        for(int i=0;i<lines_down.size();i++)
        {
        float rho=lines_down[i][0],theta=lines_down[i][1];
        double k=-(cos(theta)/sin(theta));
        double b=rho/sin(theta);
        for(int j=0;j<point.size();j++)
        {
            double distance=fabs(point.at(j).y-k*point.at(j).x-b)/sqrt(k*k+1);
            if(distance<5)
            fittedPointNumDown[i]++;
        }
        }
        findMaxAndIndex(fittedPointNumDown,lines_down.size(),MaxFittedPointNumOnLineDown,mostFittedLineDownIndex);
    }
    //计算所得到的两条最优直线的斜率和截距    
    if(MaxFittedPointNumOnLineUp>200)  
    {
            float rho=lines_up[mostFittedLineUpIndex][0],theta=lines_up[mostFittedLineUpIndex][1];
            double k=-(cos(theta)/sin(theta));
            double b=rho/sin(theta);
            twoLine_K[0]=k;
            twoLine_b[0]=b;
    }
    if(MaxFittedPointNumOnLineDown>200)  
    {
            float rho=lines_down[mostFittedLineDownIndex][0],theta=lines_down[mostFittedLineDownIndex][1];
            double k=-(cos(theta)/sin(theta));
            double b=rho/sin(theta);
            twoLine_K[1]=k;
            twoLine_b[1]=b;
    }
    //判断是拐角还是直线
    if(fabs(twoLine_b[0]-twoLine_b[1])<30)
    {
        if(MaxFittedPointNumOnLineDown>MaxFittedPointNumOnLineUp)
        {
            isused[0]=0;
            isused[1]=1;
        }
        if(MaxFittedPointNumOnLineDown<MaxFittedPointNumOnLineUp)
        {
            isused[0]=1;
            isused[1]=0;
        }
    }
    else
    {
        isused[0]=1;
        isused[1]=1; 
    }
    if(isused[0])
    {
        for(int i=0;i<point.size();i++)
        {
            double distance=fabs(point.at(i).y-twoLine_K[0]*point.at(i).x-twoLine_b[0])/sqrt(twoLine_K[0]*twoLine_K[0]+1);
            if(distance<5)
            {
                pointVector.push_back(point.at(i));
            }
        }
    }
    if(isused[1])
    {
        for(int i=0;i<point.size();i++)
        {
            double distance=fabs(point.at(i).y-twoLine_K[1]*point.at(i).x-twoLine_b[1])/sqrt(twoLine_K[1]*twoLine_K[1]+1);
            if(distance<5)
            {
                pointVector2.push_back(point.at(i));
            }
        }
    }    
    
    //根据直线拟合的结果对原始数据进行修补
      cv::Vec4f fitLine;
      if(pointVector.size()>0)
      {
        cv::fitLine(pointVector,fitLine,cv::DIST_L2,0,0.01,0.01);
        float k = fitLine[1]/fitLine[0];
        float b = fitLine[3] - k * fitLine[2];
        for(int i=0;i<pointVector.size()-1;i++)
        {
          if(pointVector.at(i+1).x-pointVector.at(i).x>1)
          {
            double x=pointVector.at(i).x+1;
            pointVector.insert(pointVector.begin()+i+1,cv::Point2d(x,x*k+b));
          }
        } 
      }
      if(pointVector2.size()>0)
      {
        cv::fitLine(pointVector2,fitLine,cv::DIST_L2,0,0.01,0.01);
        float k = fitLine[1]/fitLine[0];
        float b = fitLine[3] - k * fitLine[2];
        for(int i=0;i<pointVector2.size()-1;i++)
        {
          if(pointVector2.at(i+1).x-pointVector2.at(i).x>1)
          {
            double x=pointVector2.at(i).x+1;
            pointVector2.insert(pointVector2.begin()+i+1,cv::Point2d(x,x*k+b));
          }
        } 
      }
        if(pointVector.size()>0) 
        firstVector.push_back(pointVector);
      if(pointVector2.size()>0) 
        firstVector.push_back(pointVector2);    
      for (size_t j = 0; j < firstVector.size(); j++)
      {
            if(firstVector[j].size()>2)
            {
                cv::fitLine(firstVector[j],fitLine,cv::DIST_L2,0,0.01,0.01);

                float a = fitLine[1]/fitLine[0];
                float b = fitLine[3] - a * fitLine[2];
            
                twoLines.x1 = IMAGE_WIDTH;
                twoLines.x2 = 0;
                twoLines.y1 = IMAGE_WIDTH * a + b;
                twoLines.y2 = b;
                twoLines.lineSlop = a;
                twoLines.intercept = b;
                twoLines.lineAngle = atan(fabs(twoLines.lineSlop)) * 180 / CV_PI;
                lines->push_back(twoLines);
                //line(showIMage, cv::Point(IMAGE_WIDTH, IMAGE_WIDTH*a+b), cv::Point(0, b), cv::Scalar(255,0,0), 3);
            }
    }
}  
void RobotLocator::showImage(void)
{
        cv::imshow("src", srcImage);
        //cv::imshow("HSVImage", HSVImage);
        //cv::imshow("allBallImage", allBallImage);
        cv::imshow("afterLine", afterLine);
        cv::waitKey(1);
}
RobotLocator::~RobotLocator()
{
}