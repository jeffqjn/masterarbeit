#include <angular_v.h>
void Image::add_raw_image(sensor_msgs::ImageConstPtr m, Parameters parameters)
{
    if(m->header.stamp.toSec()>=parameters.get_START_COMPUTE_TIME()&&m->header.stamp.toSec()<=parameters.get_END_COMPUTE_TIME())
    {
        cv_bridge::CvImagePtr  image1=cv_bridge::toCvCopy( m,sensor_msgs::image_encodings::RGB8);
        cv::Mat grayscale;
        cv::Mat temp;
        temp=image1->image;
        //Grayscale image conversion
        cv::cvtColor(temp,grayscale,CV_RGB2GRAY);
        Raw_Image.emplace_back(make_pair(m->header.stamp.toSec(),grayscale));
        //
    }
    else if (m->header.stamp.toSec()>parameters.get_END_COMPUTE_TIME()&&(m->header.stamp.toSec()-parameters.get_END_COMPUTE_TIME()<0.1))
    {
        cv_bridge::CvImagePtr  image1=cv_bridge::toCvCopy( m,sensor_msgs::image_encodings::RGB8);
        cv::Mat grayscale;
        cv::Mat temp;
        temp=image1->image;
        //Grayscale image conversion
        cv::cvtColor(temp,grayscale,CV_RGB2GRAY);
        Raw_Image.emplace_back(make_pair(m->header.stamp.toSec(),grayscale));
    }
}
int Image::find_stationary() {
    cv::SurfFeatureDetector surfFeatureDetector(2000);
    cv::SurfDescriptorExtractor surfDescriptorExtractor;
    vector<pair<double,cv::Mat>> keypoints;
    for( auto image : Raw_Image)
    {
        vector<cv::KeyPoint> KeyPointTemp;
        cv::Mat temp;
        cv::FlannBasedMatcher matcher;
        vector<vector<cv::DMatch>> matchPoints;
        vector<cv::DMatch> GoodMatchPoints;
        surfFeatureDetector.detect(image.second,KeyPointTemp);
        surfDescriptorExtractor.compute(image.second,KeyPointTemp,temp);
        if(keypoints.empty())
        {
            keypoints.emplace_back(make_pair(image.first,temp));
        }
        else
        {
            vector<cv::Mat> train_desc (1, Raw_Image.back().second);
            matcher.add(train_desc);
            matcher.train();

            matcher.knnMatch(temp,matchPoints,2);
            cout<<"total match Points: "<<matchPoints.size()<<endl;

        }

    }





}
//
// Created by noah on 24.01.21.
//

