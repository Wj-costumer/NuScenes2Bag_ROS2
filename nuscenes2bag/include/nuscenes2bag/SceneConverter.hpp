#pragma once

#include "nuscenes2bag/Filesystem.hpp"
#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/FileProgress.hpp"
#include "rosbag2_cpp/writer.hpp"


namespace nuscenes2bag {

class SceneConverter {
    public:
    SceneConverter(const MetaDataProvider& metaDataProvider);

    void submit(const Token& sceneToken, FileProgress& fileProgress);

    void run(const fs::path& inPath, const fs::path& outDirectoryPath, FileProgress& fileProgress);

    private:
    void convertSampleDatas(std::unique_ptr<rosbag2_cpp::Writer>& outBag, const fs::path &inPath, FileProgress& fileProgress);
    void convertEgoPoseInfos(std::unique_ptr<rosbag2_cpp::Writer>& outBag, const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfo);
    
    private:
    const MetaDataProvider& metaDataProvider;
    std::vector<SampleDataInfo> sampleDatas;
    std::vector<EgoPoseInfo> egoPoseInfos;
    SceneId sceneId;
    Token sceneToken;
};

}