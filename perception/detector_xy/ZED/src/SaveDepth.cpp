#include "SaveDepth.hpp"

using namespace sl;
using namespace std;

int count_save = 0;
int mode_PointCloud = 0;
int mode_Depth = 0;
POINT_CLOUD_FORMAT PointCloud_format;
DEPTH_FORMAT Depth_format;

std::string getPointCloudFormatName(POINT_CLOUD_FORMAT f) {
    std::string str_;
    switch (f) {
        case POINT_CLOUD_FORMAT_XYZ_ASCII:
        str_ = "XYZ";
        break;
        case  POINT_CLOUD_FORMAT_PCD_ASCII:
        str_ = "PCD";
        break;
        case  POINT_CLOUD_FORMAT_PLY_ASCII:
        str_ = "PLY";
        break;
        case  POINT_CLOUD_FORMAT_VTK_ASCII:
        str_ = "VTK";
        break;
        default:
        break;
    }
    return str_;
}

std::string getDepthFormatName(DEPTH_FORMAT f) {
    std::string str_;
    switch (f) {
        case  DEPTH_FORMAT_PNG:
        str_ = "PNG";
        break;
        case  DEPTH_FORMAT_PFM:
        str_ = "PFM";
        break;
        case  DEPTH_FORMAT_PGM:
        str_ = "PGM";
        break;
        default:
        break;
    }
    return str_;
}

void processKeyEvent(Camera& zed, char &key) {
    switch (key) {
        case 'd':
        case 'D':
        saveDepth(zed, path + prefixDepth + to_string(count_save));
        break;

        case 'n': // Depth format
        case 'N':
        {
            mode_Depth++;
            Depth_format = static_cast<DEPTH_FORMAT> (mode_Depth % 3);
            std::cout << "Depth format: " << getDepthFormatName(Depth_format) << std::endl;
        }
        break;

        case 'p':
        case 'P':
        savePointCloud(zed, path + prefixPointCloud + to_string(count_save));
        break;


        case 'm': // Point cloud format
        case 'M':
        {
            mode_PointCloud++;
            PointCloud_format = static_cast<POINT_CLOUD_FORMAT> (mode_PointCloud % 4);
            std::cout << "Point Cloud format: " << getPointCloudFormatName(PointCloud_format) << std::endl;
        }
        break;

        case 'h': // Print help
        case 'H':
        cout << helpString << endl;
        break;

        case 's': // Save side by side image
        saveSbSImage(zed, std::string("ZED_image") + std::to_string(count_save) + std::string(".png"));
        break;
    }
    count_save++;
}

void savePointCloud(Camera& zed, std::string filename) {
    std::cout << "Saving Point Cloud... " << flush;
    bool saved = savePointCloudAs(zed, PointCloud_format, filename.c_str(), true);
    if (saved)
        std::cout << "Done" << endl;
    else
        std::cout << "Failed... Please check that you have permissions to write on disk" << endl;
}

void saveDepth(Camera& zed, std::string filename) {
    float max_value = std::numeric_limits<unsigned short int>::max();
    float scale_factor = max_value / zed.getDepthMaxRangeValue();

    std::cout << "Saving Depth Map... " << flush;
    bool saved = saveDepthAs(zed, Depth_format, filename.c_str(), scale_factor);
    if (saved)
        std::cout << "Done" << endl;
    else
        std::cout << "Failed... Please check that you have permissions to write on disk" << endl;
}

void saveSbSImage(Camera& zed, std::string filename) {
    Resolution image_size = zed.getResolution();

    cv::Mat sbs_image(image_size.height, image_size.width * 2, CV_8UC4);
    cv::Mat left_image(sbs_image, cv::Rect(0, 0, image_size.width, image_size.height));
    cv::Mat right_image(sbs_image, cv::Rect(image_size.width, 0, image_size.width, image_size.height));

    Mat buffer_sl;
    cv::Mat buffer_cv;

    zed.retrieveImage(buffer_sl, VIEW_LEFT);
    buffer_cv = cv::Mat(buffer_sl.getHeight(), buffer_sl.getWidth(), CV_8UC4, buffer_sl.getPtr<sl::uchar1>(MEM_CPU));
    buffer_cv.copyTo(left_image);
    zed.retrieveImage(buffer_sl, VIEW_RIGHT);
    buffer_cv = cv::Mat(buffer_sl.getHeight(), buffer_sl.getWidth(), CV_8UC4, buffer_sl.getPtr<sl::uchar1>(MEM_CPU));
    buffer_cv.copyTo(right_image);

    cv::cvtColor(sbs_image, sbs_image, CV_RGBA2RGB);
    cv::imwrite(filename, sbs_image);
}
