/*
 * @Author: Xiawenlong-bug 2473833028@qq.com
 * @Date: 2024-02-13 21:30:17
 * @LastEditors: Xiawenlong-bug 2473833028@qq.com
 * @LastEditTime: 2024-02-14 15:51:45
 * @FilePath: /myORBSLAM2/include/ORBextractor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef ORBEXTRACT_H
#define ORBEXTRACT_H

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>

namespace ORB_SLAM2
{
// 提取器节点
class ExtractorNode
{
    public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    	///保存有当前节点的特征点
    std::vector<cv::KeyPoint> vKeys;
	///当前节点所对应的图像坐标边界
    cv::Point2i UL, UR, BL, BR;
	//存储提取器节点的列表（其实就是双向链表）的一个迭代器,可以参考[http://www.runoob.com/cplusplus/cpp-overloading.html]
	//这个迭代器提供了访问总节点列表的方式，需要结合cpp文件进行分析
    std::list<ExtractorNode>::iterator lit;
	
	///如果节点中只有一个特征点的话，说明这个节点不能够再进行分裂了，这个标志置位
	//这个节点中如果没有特征点的话，这个节点就直接被删除了
    bool bNoMore;
};

class ORBextractor
{
    public:
    //TODO 但是在程序中好像并没有被用到
	///定义一个枚举类型用于表示使用HARRIS响应值还是使用FAST响应值
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /**
     * @brief 构造函数
     * @detials 之所以会有两种响应值的阈值，原因是，程序先使用初始的默认FAST响应值阈值提取图像cell中的特征点；如果提取到的
     * 特征点数目不足，那么就降低要求，使用较小FAST响应值阈值进行再次提取，以获得尽可能多的FAST角点。
     * @param[in] nfeatures         指定要提取出来的特征点数目
     * @param[in] scaleFactor       图像金字塔的缩放系数
     * @param[in] nlevels           指定需要提取特征点的图像金字塔层
     * @param[in] iniThFAST         初始的默认FAST响应值阈值
     * @param[in] minThFAST         较小的FAST响应值阈值
     */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);

    ~ORBextractor();

    
    void operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,cv::OutputArray descriptors);


	//下面的这些内联函数都是用来直接获取类的成员变量的
	
    /**
     * @brief 获取图像金字塔的层数
     * @return int 图像金字塔的层数
     */
    int inline GetLevels(){
        return nlevels;}

    /**
     * @brief 获取当前提取器所在的图像的缩放因子，这个不带s的因子表示是相临近层之间的
     * @return float 当前提取器所在的图像的缩放因子，相邻层之间
     */
    float inline GetScaleFactor(){
        return scaleFactor;}

    /**
     * @brief 获取图像金字塔中每个图层相对于底层图像的缩放因子
     * @return std::vector<float> 图像金字塔中每个图层相对于底层图像的缩放因子
     */
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /**
     * @brief 获取上面的那个缩放因子s的倒数
     * @return std::vector<float> 倒数
     */
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }
    
    /**
     * @brief 获取sigma^2，就是每层图像相对于初始图像缩放因子的平方，参考cpp文件中类构造函数的操作
     * @return std::vector<float> sigma^2
     */
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    /**
     * @brief 获取上面sigma平方的倒数
     * @return std::vector<float> 
     */
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    ///这个是用来存储图像金字塔的变量，一个元素存储一层图像
    std::vector<cv::Mat> mvImagePyramid;

protected:
    void ComputePyramid(cv::Mat image);

    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);

    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX, const int &maxX, const int &minY, const int &maxY,
                                                const int &nFeatures, const int &level);

    std::vector<cv::Point> pattern;

    int nfeatures;			                    ///<整个图像金字塔中，要提取的特征点数目
    double scaleFactor;		                    ///<图像金字塔层与层之间的缩放因子
    int nlevels;			                    ///<图像金字塔的层数
    int iniThFAST;			                    ///<初始的FAST响应值阈值
    int minThFAST;			                    ///<最小的FAST响应值阈值

    std::vector<int> mnFeaturesPerLevel;		///<分配到每层图像中，要提取的特征点数目

    std::vector<int> umax;	                    ///<计算特征点方向的时候，有个圆形的图像区域，这个vector中存储了每行u轴的边界（四分之一，其他部分通过对称获得）

    std::vector<float> mvScaleFactor;		    ///<每层图像的缩放因子
    std::vector<float> mvInvScaleFactor;        ///<以及每层缩放因子的倒数
    std::vector<float> mvLevelSigma2;		    ///<存储每层的sigma^2,即上面每层图像相对于底层图像缩放倍数的平方
    std::vector<float> mvInvLevelSigma2;	    ///<sigma平方的倒数
};
}
#endif
