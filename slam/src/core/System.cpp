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


#include "core/System.h"
#include <iomanip>
#include <thread>

#include "utils/Converter.h"

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const std::string& strVocFile,
               const std::string& strSettingsFile,
               const eSensor      sensor,
               const bool         bUseViewer,
               const int          initFr,
               const std::string& strSequence)
    : mSensor(sensor)
    , mbReset(false)
    , mbResetActiveMap(false)
    , mbActivateLocalizationMode(false)
    , mbDeactivateLocalizationMode(false)
    , mbShutDown(false)
{
    // Output welcome message
    std::cout
        << std::endl
        << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, "
           "Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University "
           "of Zaragoza."
        << std::endl
        << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. "
           "Montiel and Juan D. Tardós, University of Zaragoza."
        << std::endl
        << "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl
        << "This is free software, and you are welcome to redistribute it"
        << std::endl
        << "under certain conditions. See LICENSE.txt." << std::endl
        << std::endl;

    std::cout << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
        std::cout << "Monocular" << std::endl;
    else if (mSensor == STEREO)
        std::cout << "Stereo" << std::endl;
    else if (mSensor == RGBD)
        std::cout << "RGB-D" << std::endl;
    else if (mSensor == IMU_MONOCULAR)
        std::cout << "Monocular-Inertial" << std::endl;
    else if (mSensor == IMU_STEREO)
        std::cout << "Stereo-Inertial" << std::endl;
    else if (mSensor == IMU_RGBD)
        std::cout << "RGB-D-Inertial" << std::endl;

    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "Failed to open settings file at: " << strSettingsFile
                  << std::endl;
        exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if (!node.empty() && node.isString() && node.string() == "1.0")
    {
        settings_ = new Settings(strSettingsFile, mSensor);

        mStrLoadAtlasFromFile = settings_->atlasLoadFile();
        mStrSaveAtlasToFile   = settings_->atlasSaveFile();

        std::cout << "Settings is set up from the setting file." << std::endl;
        std::cout << (*settings_) << std::endl;
    }
    else
    {
        settings_ = nullptr;
        std::cout << "Settings is set to nullptr." << std::endl;

        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if (!node.empty() && node.isString())
        {
            mStrLoadAtlasFromFile = (std::string)node;
        }

        node = fsSettings["System.SaveAtlasToFile"];
        if (!node.empty() && node.isString())
        {
            mStrSaveAtlasToFile = (std::string)node;
        }
    }

    node          = fsSettings["loopClosing"];
    bool activeLC = true;
    if (!node.empty())
    {
        activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    mStrVocabularyFilePath = strVocFile;

    bool loadedAtlas = false;

    if (mStrLoadAtlasFromFile.empty())
    {
        // Load ORB Vocabulary
        std::cout << std::endl
                  << "Loading ORB Vocabulary. This could take a while..."
                  << std::endl;

        mpVocabulary  = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if (!bVocLoad)
        {
            std::cerr << "Wrong path to vocabulary. " << std::endl;
            std::cerr << "Falied to open at: " << strVocFile << std::endl;
            exit(-1);
        }
        std::cout << "Vocabulary loaded!" << std::endl << std::endl;

        // Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        // Create the Atlas
        std::cout << "Initialization of Atlas from scratch " << std::endl;
        mpAtlas = new Atlas(0);
    }
    else
    {
        throw std::runtime_error("Not supported atlas type.");
    }

    if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR
        || mSensor == IMU_RGBD)
        mpAtlas->SetInertialSensor();

    // Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this
    // constructor)
    std::cout << "Seq. Name: " << strSequence << std::endl;
    mpTracker = new Tracking(this,
                             mpVocabulary,
                             mpAtlas,
                             mpKeyFrameDatabase,
                             strSettingsFile,
                             mSensor,
                             settings_,
                             strSequence);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper =
        new LocalMapping(this,
                         mpAtlas,
                         mSensor == MONOCULAR || mSensor == IMU_MONOCULAR,
                         mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO
                             || mSensor == IMU_RGBD,
                         strSequence);
    mptLocalMapping =
        new std::thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
    mpLocalMapper->mInitFr = initFr;
    if (settings_)
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if (mpLocalMapper->mThFarPoints != 0)
    {
        std::cout << "Discard points further than "
                  << mpLocalMapper->mThFarPoints << " m from current camera"
                  << std::endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    // Initialize the Loop Closing thread and launch
    //  mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas,
                                   mpKeyFrameDatabase,
                                   mpVocabulary,
                                   mSensor != MONOCULAR,
                                   activeLC);  // mSensor!=MONOCULAR);
    mptLoopClosing =
        new std::thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // usleep(10*1000*1000);

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);
}

System::System(ORBVocabulary* vocabulary,
               Settings*      settings,
               const eSensor  sensor)
    : mSensor(sensor)
    , mpVocabulary(vocabulary)
    , mbReset(false)
    , mbResetActiveMap(false)
    , mbActivateLocalizationMode(false)
    , mbDeactivateLocalizationMode(false)
    , mbShutDown(false)
    , settings_(settings)
{
    // components constructed by vocabulary
    {
        // Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        assert(mpKeyFrameDatabase && "Failed to create Keyframe database.");
        std::cout << "Keyframe database has been created." << std::endl;

        // atlas is not loaded from file
        mpAtlas = new Atlas(0);
        assert(mpAtlas && "Failed to create atlas.");
        std::cout << "Atlas has been created." << std::endl;
    }

    // setup imu sensor
    {
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO
            || mSensor == IMU_RGBD)
        {
            mpAtlas->SetInertialSensor();
            std::cout << "Inertial sensor has been." << std::endl;
        }
    }

    {
        // create tracker
        mpTracker = new Tracking(this,
                                 mpVocabulary,
                                 mpAtlas,
                                 mpKeyFrameDatabase,
                                 "",
                                 mSensor,
                                 settings_,
                                 "");
        std::cout << "Tracker has been created." << std::endl;


        // create local mapper and its thread
        bool bMonocular = (mSensor == MONOCULAR) || (mSensor == IMU_MONOCULAR);
        bool bImu       = (mSensor == IMU_MONOCULAR) || (mSensor == IMU_STEREO)
                    || (mSensor == IMU_RGBD);
        mpLocalMapper = new LocalMapping(this, mpAtlas, bMonocular, bImu, "");
        assert(mpLocalMapper && "Failed to create local mapper.");
        std::cout << "Local mapper has been created." << std::endl;
        mptLocalMapping =
            new std::thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
        std::cout << "Local mapper thread has been created." << std::endl;
        mpLocalMapper->mInitFr      = 0;
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
        if (mpLocalMapper->mThFarPoints != 0)
        {
            std::cout << "Discard points further than "
                      << mpLocalMapper->mThFarPoints << " m from current camera"
                      << std::endl;
            mpLocalMapper->mbFarPoints = true;
        }
        else
        {
            mpLocalMapper->mbFarPoints = false;
        }


        // create loop closer and its thread
        bool bActiveLoopClosing = true;
        mpLoopCloser =
            new LoopClosing(mpAtlas,
                            mpKeyFrameDatabase,
                            mpVocabulary,
                            mSensor != MONOCULAR,
                            bActiveLoopClosing);  // mSensor!=MONOCULAR);
        mptLoopClosing =
            new std::thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);
        std::cout << "Loop closer has been created." << std::endl;
    }

    {
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    {
        Verbose::SetTh(Verbose::VERBOSITY_QUIET);
    }
}

Tracking& System::getTracker() const
{
    return *mpTracker;
}

Sophus::SE3f System::TrackStereo(const cv::Mat&                 imLeft,
                                 const cv::Mat&                 imRight,
                                 const double&                  timestamp,
                                 const std::vector<IMU::Point>& vImuMeas,
                                 std::string                    filename)
{
    if (mSensor != STEREO && mSensor != IMU_STEREO)
    {
        std::cerr
            << "ERROR: you called TrackStereo but input sensor was not set to "
               "Stereo nor Stereo-Inertial."
            << std::endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if (settings_ && settings_->needToRectify())
    {
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if (settings_ && settings_->needToResize())
    {
        cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
        cv::resize(imRight, imRightToFeed, settings_->newImSize());
    }
    else
    {
        imLeftToFeed  = imLeft.clone();
        imRightToFeed = imRight.clone();
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset          = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    // std::cout << "start GrabImageStereo" << std::endl;
    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(
        imLeftToFeed, imRightToFeed, timestamp, filename);

    // std::cout << "out grabber" << std::endl;

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState      = mpTracker->mState;
    mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat&                 im,
                               const cv::Mat&                 depthmap,
                               const double&                  timestamp,
                               const std::vector<IMU::Point>& vImuMeas,
                               std::string                    filename)
{
    if (mSensor != RGBD && mSensor != IMU_RGBD)
    {
        std::cerr
            << "ERROR: you called TrackRGBD but input sensor was not set to "
               "RGBD."
            << std::endl;
        exit(-1);
    }

    cv::Mat imToFeed      = im.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if (settings_ && settings_->needToResize())
    {
        cv::Mat resizedIm;
        cv::resize(im, resizedIm, settings_->newImSize());
        imToFeed = resizedIm;

        cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset          = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_RGBD)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw =
        mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState      = mpTracker->mState;
    mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat&                 im,
                                    const double&                  timestamp,
                                    const std::vector<IMU::Point>& vImuMeas,
                                    std::string                    filename)
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbShutDown)
        {
            return Sophus::SE3f();
        }
    }

    if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR)
    {
        std::cerr
            << "ERROR: you called TrackMonocular but input sensor was not set "
               "to Monocular nor Monocular-Inertial."
            << std::endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if (settings_ && settings_->needToResize())
    {
        cv::Mat resizedIm;
        cv::resize(im, resizedIm, settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset          = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            std::cout << "SYSTEM-> Reseting active map in monocular case"
                      << std::endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw =
        mpTracker->GrabImageMonocular(imToFeed, timestamp, filename);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState      = mpTracker->mState;
    mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    // std::cout
    //     << "Current global map point count: [" <<
    //     mpAtlas->GetAllMapPoints().size()
    //     << "]    Current local map point count: [" <<
    //     mpAtlas->GetReferenceMapPoints().size()
    //     << "]    Current keyframe count: [" <<
    //     mpAtlas->GetAllKeyFrames().size()
    //     << "]" << std::endl;

    return Tcw;
}


void System::ActivateLocalizationMode()
{
    std::unique_lock<std::mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    std::unique_lock<std::mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n    = 0;
    int        curn = mpAtlas->GetLastBigChangeIdx();
    if (n < curn)
    {
        n = curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    std::cout << "Shutdown" << std::endl;

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();

    mptLocalMapping->join();
    mptLoopClosing->join();

    delete mptLocalMapping;
    delete mptLoopClosing;
}

bool System::isShutDown()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    return mbShutDown;
}

void System::SaveTrajectoryTUM(const std::string& filename)
{
    std::cout << std::endl
              << "Saving camera trajectory to " << filename << " ..."
              << std::endl;
    if (mSensor == MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular."
                  << std::endl;
        return;
    }

    std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFrame*>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT  = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator   lbL = mpTracker->mlbLost.begin();
    for (std::list<Sophus::SE3f>::iterator
             lit  = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        while (pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f    twc = Twc.translation();
        Eigen::Quaternionf q   = Twc.unit_quaternion();

        f << std::setprecision(6) << *lT << " " << std::setprecision(9)
          << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " "
          << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    f.close();
    // std::cout << std::endl << "trajectory saved!" << std::endl;
}

void System::SaveKeyFrameTrajectoryTUM(const std::string& filename)
{
    std::cout << std::endl
              << "Saving keyframe trajectory to " << filename << " ..."
              << std::endl;

    std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if (pKF->isBad())
            continue;

        Sophus::SE3f       Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q   = Twc.unit_quaternion();
        Eigen::Vector3f    t   = Twc.translation();
        f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7)
          << " " << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " "
          << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const std::string& filename)
{

    std::cout << std::endl
              << "Saving trajectory to " << filename << " ..." << std::endl;
    /*if(mSensor==MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular."
    << endl; return;
    }*/

    std::vector<Map*> vpMaps    = mpAtlas->GetAllMaps();
    int               numMaxKFs = 0;
    Map*              pBiggerMap;
    std::cout << "There are " << std::to_string(vpMaps.size())
              << " maps in the atlas" << std::endl;
    for (Map* pMap : vpMaps)
    {
        std::cout << "  Map " << std::to_string(pMap->GetId()) << " has "
                  << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs"
                  << std::endl;
        if (pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs  = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f
        Twb;  // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO
        || mSensor == IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    // std::cout << "file open" << std::endl;
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFrame*>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT  = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator   lbL = mpTracker->mlbLost.begin();

    // std::cout << "size mlpReferences: " << mpTracker->mlpReferences.size() <<
    // endl; std::cout << "size mlRelativeFramePoses: " <<
    // mpTracker->mlRelativeFramePoses.size() << std::endl; std::cout << "size
    // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() <<
    // std::endl; std::cout << "size mpTracker->mlbLost: " <<
    // mpTracker->mlbLost.size() << std::endl;


    for (auto lit  = mpTracker->mlRelativeFramePoses.begin(),
              lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++, lbL++)
    {
        // std::cout << "1" << std::endl;
        if (*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        // std::cout << "KF: " << pKF->mnId << std::endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        if (!pKF)
            continue;

        // std::cout << "2.5" << std::endl;

        while (pKF->isBad())
        {
            // std::cout << " 2.bad" << std::endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            // std::cout << "--Parent KF: " << pKF->mnId << std::endl;
        }

        if (!pKF || pKF->GetMap() != pBiggerMap)
        {
            // std::cout << "--Parent KF is from another map" << std::endl;
            continue;
        }

        // std::cout << "3" << std::endl;

        Trw = Trw * pKF->GetPose()
              * Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // std::cout << "4" << std::endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO
            || mSensor == IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << twb(0) << " " << twb(1) << " "
              << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f       Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    twc = Twc.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << twc(0) << " " << twc(1) << " "
              << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        }

        // std::cout << "5" << std::endl;
    }
    // std::cout << "end saving trajectory" << std::endl;
    f.close();
    std::cout << std::endl
              << "End of saving trajectory to " << filename << " ..."
              << std::endl;
}

void System::SaveTrajectoryEuRoC(const std::string& filename, Map* pMap)
{

    std::cout << std::endl
              << "Saving trajectory of map " << pMap->GetId() << " to "
              << filename << " ..." << std::endl;
    /*if(mSensor==MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular."
    << endl; return;
    }*/

    int numMaxKFs = 0;

    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f
        Twb;  // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO
        || mSensor == IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    // std::cout << "file open" << std::endl;
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFrame*>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT  = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator   lbL = mpTracker->mlbLost.begin();

    // std::cout << "size mlpReferences: " << mpTracker->mlpReferences.size() <<
    // endl; std::cout << "size mlRelativeFramePoses: " <<
    // mpTracker->mlRelativeFramePoses.size() << std::endl; std::cout << "size
    // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() <<
    // std::endl; std::cout << "size mpTracker->mlbLost: " <<
    // mpTracker->mlbLost.size() << std::endl;


    for (auto lit  = mpTracker->mlRelativeFramePoses.begin(),
              lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++, lbL++)
    {
        // std::cout << "1" << std::endl;
        if (*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        // std::cout << "KF: " << pKF->mnId << std::endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        if (!pKF)
            continue;

        // std::cout << "2.5" << std::endl;

        while (pKF->isBad())
        {
            // std::cout << " 2.bad" << std::endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            // std::cout << "--Parent KF: " << pKF->mnId << std::endl;
        }

        if (!pKF || pKF->GetMap() != pMap)
        {
            // std::cout << "--Parent KF is from another map" << std::endl;
            continue;
        }

        // std::cout << "3" << std::endl;

        Trw = Trw * pKF->GetPose()
              * Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // std::cout << "4" << std::endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO
            || mSensor == IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << twb(0) << " " << twb(1) << " "
              << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f       Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    twc = Twc.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << twc(0) << " " << twc(1) << " "
              << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        }

        // std::cout << "5" << std::endl;
    }
    // std::cout << "end saving trajectory" << std::endl;
    f.close();
    std::cout << std::endl
              << "End of saving trajectory to " << filename << " ..."
              << std::endl;
}

/*void System::SaveTrajectoryEuRoC(const string &filename)
{

    std::cout << std::endl << "Saving trajectory to " << filename << " ..." <<
std::endl; if(mSensor==MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular."
<< endl; return;
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or
not. if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD) Twb
= vpKFs[0]->GetImuPose_(); else Twb = vpKFs[0]->GetPoseInverse_();

    std::ofstream f;
    f.open(filename.c_str());
    // std::cout << "file open" << std::endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit =
mpTracker->mlpReferences.begin(); list<double>::iterator lT =
mpTracker->mlFrameTimes.begin(); list<bool>::iterator lbL =
mpTracker->mlbLost.begin();

    //std::cout << "size mlpReferences: " << mpTracker->mlpReferences.size() <<
std::endl;
    //std::cout << "size mlRelativeFramePoses: " <<
mpTracker->mlRelativeFramePoses.size() << std::endl;
    //std::cout << "size mpTracker->mlFrameTimes: " <<
mpTracker->mlFrameTimes.size()
<< std::endl;
    //std::cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() <<
std::endl;


    for(list<Sophus::SE3f>::iterator
lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++,
lT++, lbL++)
    {
        //std::cout << "1" << std::endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //std::cout << "KF: " << pKF->mnId << std::endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
get a suitable keyframe. if (!pKF) continue;

        //std::cout << "2.5" << std::endl;

        while(pKF->isBad())
        {
            //std::cout << " 2.bad" << std::endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //std::cout << "--Parent KF: " << pKF->mnId << std::endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //std::cout << "--Parent KF is from another map" << std::endl;
            continue;
        }

        //std::cout << "3" << std::endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new
world reference

        // std::cout << "4" << std::endl;


        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
mSensor==IMU_RGBD)
        {
            Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
            Sophus::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<
std::setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x()
<< " " << q.y() << " "
<< q.z() << " " << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<
std::setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x()
<< " " << q.y() << " "
<< q.z() << " " << q.w() << std::endl;
        }

        // std::cout << "5" << std::endl;
    }
    //std::cout << "end saving trajectory" << std::endl;
    f.close();
    std::cout << std::endl << "End of saving trajectory to " << filename << "
..." << endl;
}*/


/*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
{
    std::cout << std::endl << "Saving keyframe trajectory to " << filename << "
..."
<< endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
mSensor==IMU_RGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<
std::setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " <<
twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<
endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp << " " <<
std::setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " <<
t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<
endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const std::string& filename)
{
    std::cout << std::endl
              << "Saving keyframe trajectory to " << filename << " ..."
              << std::endl;

    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map*              pBiggerMap;
    int               numMaxKFs = 0;
    for (Map* pMap : vpMaps)
    {
        if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs  = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if (!pBiggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if (!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO
            || mSensor == IMU_RGBD)
        {
            Sophus::SE3f       Twb = pKF->GetImuPose();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << std::setprecision(9) << twb(0) << " " << twb(1) << " "
              << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f       Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    t   = Twc.translation();
            f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << std::setprecision(9) << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;
        }
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const std::string& filename, Map* pMap)
{
    std::cout << std::endl
              << "Saving keyframe trajectory of map " << pMap->GetId() << " to "
              << filename << " ..." << std::endl;

    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if (!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO
            || mSensor == IMU_RGBD)
        {
            Sophus::SE3f       Twb = pKF->GetImuPose();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << std::setprecision(9) << twb(0) << " " << twb(1) << " "
              << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f       Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    t   = Twc.translation();
            f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << std::setprecision(9) << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;
        }
    }
    f.close();
}

/*void System::SaveTrajectoryKITTI(const string &filename)
{
    std::cout << std::endl << "Saving camera trajectory to " << filename << "
..." << endl; if(mSensor==MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular."
<< endl; return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit =
mpTracker->mlpReferences.begin(); list<double>::iterator lT =
mpTracker->mlFrameTimes.begin(); for(list<cv::Mat>::iterator
lit=mpTracker->mlRelativeFramePoses.begin(),
lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << std::setprecision(9) << Rwc.at<float>(0,0) << " " <<
Rwc.at<float>(0,1)
<< " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " <<
Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " << Rwc.at<float>(2,0) << "
" << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  <<
twc.at<float>(2) << std::endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const std::string& filename)
{
    std::cout << std::endl
              << "Saving camera trajectory to " << filename << " ..."
              << std::endl;
    if (mSensor == MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular."
                  << std::endl;
        return;
    }

    std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFrame*>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (std::list<Sophus::SE3f>::iterator
             lit  = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        if (!pKF)
            continue;

        while (pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Tow;

        Sophus::SE3f    Tcw = (*lit) * Trw;
        Sophus::SE3f    Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << std::setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " "
          << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1)
          << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " "
          << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << std::endl;
    }
    f.close();
}


void System::SaveDebugData(const int& initIdx)
{
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_"
                        + std::to_string(mpLocalMapper->mInitSect) + "_"
                        + std::to_string(initIdx) + ".txt");

    // 1. Save scale
    std::ofstream f;
    f.open("init_Scale_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mScale << std::endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << ","
      << mpLocalMapper->mRwg(0, 2) << std::endl;
    f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << ","
      << mpLocalMapper->mRwg(1, 2) << std::endl;
    f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << ","
      << mpLocalMapper->mRwg(2, 2) << std::endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mCostTime << std::endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << ","
      << mpLocalMapper->mbg(2) << std::endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << ","
      << mpLocalMapper->mba(2) << std::endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" + std::to_string(mpLocalMapper->mInitSect) + "_"
               + std::to_string(initIdx) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++)
    {
        for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++)
        {
            if (j != 0)
                f << ",";
            f << std::setprecision(15) << mpLocalMapper->mcovInertial(i, j);
        }
        f << std::endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mInitTime << std::endl;
    f.close();
}


int System::GetTrackingState()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackingState;
}

std::vector<MapPoint*> System::GetTrackedMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    if ((aux > 0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        // if ((mpTracker->mState==Tracking::LOST))
        // //||(mpTracker->mState==Tracking::RECENTLY_LOST))
        //     return true;
        // else
        //     return false;
        return mpTracker->mState == Tracking::LOST;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit() > 0.1);
}

void System::ChangeDataset()
{
    if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

float System::GetImageScale()
{
    return mpTracker->GetImageScale();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time)
{
    mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif


}  // namespace ORB_SLAM3
