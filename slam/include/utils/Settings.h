/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H
#include <iostream>
#include <optional>
#include <string>

#include "camera_models/GeometricCamera.h"

namespace ORB_SLAM3
{

class System;

// TODO: change to double instead of float

class Settings
{
public:
    /*
     * Enum for the different camera types implemented
     */
    enum CameraType
    {
        PinHole       = 0,
        Rectified     = 1,
        KannalaBrandt = 2
    };

    struct SettingDesc
    {
        int sensor;

        struct
        {
            CameraType cameraType;
            float      fx;
            float      fy;
            float      cx;
            float      cy;
        } cameraInfo;

        struct DistortionParam
        {
            float                k1;
            float                k2;
            float                p1;
            float                p2;
            std::optional<float> k3;
        };
        std::optional<DistortionParam> distortion;

        struct
        {
            int32_t width;
            int32_t height;

            std::optional<int32_t> newWidth;
            std::optional<int32_t> newHeight;

            float fps;
            bool  bRGB = false;
        } imageInfo;

        struct
        {
            float   noiseGyro;
            float   noiseAcc;
            float   gyroWalk;
            float   accWalk;
            float   frequency;
            cv::Mat cvTbc;
            bool    bInsertKFsWhenLost = true;
        } imuInfo;

        struct
        {
            int32_t nFeatures;
            float   scaleFactor;
            int32_t nLevels;
            int32_t initThFAST;
            int32_t minThFAST;
        } orbInfo;

        struct
        {
            float keyframeSize;
            float keyframeLineWidth;
            float graphLineWidth;
            float pointSize;
            float cameraSize;
            float cameraLineWidth;
            float viewPointX;
            float viewPointY;
            float viewPointZ;
            float viewPointF;
            float imageViewerScale = 1.0f;
        } viewerInfo;

        struct
        {
            std::string loadPath;
            std::string savePath;
        } atlasInfo;

        struct
        {
            float thFarPoints = 0.0f;
        } otherInfo;
    };

    Settings(const SettingDesc& desc);

    /*
     * Constructor from file
     */
    Settings(const std::string& configFile, const int& sensor);

    /*
     * Ostream operator overloading to dump settings to the terminal
     */
    friend std::ostream& operator<<(std::ostream& output, const Settings& s);

    /*
     * Getter methods
     */
    CameraType cameraType()
    {
        return cameraType_;
    }
    GeometricCamera* camera1()
    {
        return calibration1_;
    }
    GeometricCamera* camera2()
    {
        return calibration2_;
    }
    cv::Mat camera1DistortionCoef()
    {
        return cv::Mat(vPinHoleDistorsion1_.size(),
                       1,
                       CV_32F,
                       vPinHoleDistorsion1_.data());
    }
    cv::Mat camera2DistortionCoef()
    {
        return cv::Mat(vPinHoleDistorsion2_.size(),
                       1,
                       CV_32F,
                       vPinHoleDistorsion1_.data());
    }

    Sophus::SE3f Tlr()
    {
        return Tlr_;
    }
    float bf()
    {
        return bf_;
    }
    float b()
    {
        return b_;
    }
    float thDepth()
    {
        return thDepth_;
    }

    bool needToUndistort()
    {
        return bNeedToUndistort_;
    }

    cv::Size newImSize()
    {
        return newImSize_;
    }
    float fps()
    {
        return fps_;
    }
    bool rgb()
    {
        return bRGB_;
    }
    bool needToResize()
    {
        return bNeedToResize1_;
    }
    bool needToRectify()
    {
        return bNeedToRectify_;
    }

    float noiseGyro()
    {
        return noiseGyro_;
    }
    float noiseAcc()
    {
        return noiseAcc_;
    }
    float gyroWalk()
    {
        return gyroWalk_;
    }
    float accWalk()
    {
        return accWalk_;
    }
    float imuFrequency()
    {
        return imuFrequency_;
    }
    Sophus::SE3f Tbc()
    {
        return Tbc_;
    }
    bool insertKFsWhenLost()
    {
        return insertKFsWhenLost_;
    }

    float depthMapFactor()
    {
        return depthMapFactor_;
    }

    int nFeatures()
    {
        return nFeatures_;
    }
    int nLevels()
    {
        return nLevels_;
    }
    float initThFAST()
    {
        return initThFAST_;
    }
    float minThFAST()
    {
        return minThFAST_;
    }
    float scaleFactor()
    {
        return scaleFactor_;
    }

    float keyFrameSize()
    {
        return keyFrameSize_;
    }
    float keyFrameLineWidth()
    {
        return keyFrameLineWidth_;
    }
    float graphLineWidth()
    {
        return graphLineWidth_;
    }
    float pointSize()
    {
        return pointSize_;
    }
    float cameraSize()
    {
        return cameraSize_;
    }
    float cameraLineWidth()
    {
        return cameraLineWidth_;
    }
    float viewPointX()
    {
        return viewPointX_;
    }
    float viewPointY()
    {
        return viewPointY_;
    }
    float viewPointZ()
    {
        return viewPointZ_;
    }
    float viewPointF()
    {
        return viewPointF_;
    }
    float imageViewerScale()
    {
        return imageViewerScale_;
    }

    std::string atlasLoadFile()
    {
        return sLoadFrom_;
    }
    std::string atlasSaveFile()
    {
        return sSaveto_;
    }

    float thFarPoints()
    {
        return thFarPoints_;
    }

    cv::Mat M1l()
    {
        return M1l_;
    }
    cv::Mat M2l()
    {
        return M2l_;
    }
    cv::Mat M1r()
    {
        return M1r_;
    }
    cv::Mat M2r()
    {
        return M2r_;
    }

    int getSensor() const
    {
        return sensor_;
    }

private:
    template <typename T>
    T readParameter(cv::FileStorage&   fSettings,
                    const std::string& name,
                    bool&              found,
                    const bool         required = true)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name
                          << " required parameter does not exist, aborting..."
                          << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..."
                          << std::endl;
                found = false;
                return T();
            }
        }
        else
        {
            found = true;
            return (T)node;
        }
    }

    void readCamera1(cv::FileStorage& fSettings);
    void readCamera2(cv::FileStorage& fSettings);
    void readImageInfo(cv::FileStorage& fSettings);
    void readIMU(cv::FileStorage& fSettings);
    void readRGBD(cv::FileStorage& fSettings);
    void readORB(cv::FileStorage& fSettings);
    void readViewer(cv::FileStorage& fSettings);
    void readLoadAndSave(cv::FileStorage& fSettings);
    void readOtherParameters(cv::FileStorage& fSettings);

    void precomputeRectificationMaps();

    int        sensor_;
    CameraType cameraType_;  // Camera type

    /*
     * Visual stuff
     */
    GeometricCamera *  calibration1_, *calibration2_;  // Camera calibration
    GeometricCamera *  originalCalib1_, *originalCalib2_;
    std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

    cv::Size originalImSize_, newImSize_;
    float    fps_;
    bool     bRGB_;

    bool bNeedToUndistort_;
    bool bNeedToRectify_;
    bool bNeedToResize1_, bNeedToResize2_;

    Sophus::SE3f Tlr_;
    float        thDepth_;
    float        bf_, b_;

    /*
     * Rectification stuff
     */
    cv::Mat M1l_, M2l_;
    cv::Mat M1r_, M2r_;

    /*
     * Inertial stuff
     */
    float        noiseGyro_, noiseAcc_;
    float        gyroWalk_, accWalk_;
    float        imuFrequency_;
    Sophus::SE3f Tbc_;
    bool         insertKFsWhenLost_;

    /*
     * RGBD stuff
     */
    float depthMapFactor_;

    /*
     * ORB stuff
     */
    int   nFeatures_;
    float scaleFactor_;
    int   nLevels_;
    int   initThFAST_, minThFAST_;

    /*
     * Viewer stuff
     */
    float keyFrameSize_;
    float keyFrameLineWidth_;
    float graphLineWidth_;
    float pointSize_;
    float cameraSize_;
    float cameraLineWidth_;
    float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
    float imageViewerScale_;

    /*
     * Save & load maps
     */
    std::string sLoadFrom_, sSaveto_;

    /*
     * Other stuff
     */
    float thFarPoints_;
};
};  // namespace ORB_SLAM3


#endif  // ORB_SLAM3_SETTINGS_H
