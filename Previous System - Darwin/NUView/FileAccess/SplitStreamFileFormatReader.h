#ifndef SPLITSTREAMFILEFORMATREADER_H
#define SPLITSTREAMFILEFORMATREADER_H
#include "LogFileFormatReader.h"
#include "StreamFileReader.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "Localisation/Localisation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUSensorsData/NULocalisationSensors.h"
#include "Localisation/LocWmFrame.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include <QDir>

class SplitStreamFileFormatReader: public LogFileFormatReader
{
    Q_OBJECT
public:
    SplitStreamFileFormatReader();
public:
    explicit SplitStreamFileFormatReader(QObject *parent = 0);
    explicit SplitStreamFileFormatReader(const QString& filename, QObject *parent = 0);
    ~SplitStreamFileFormatReader();

    int openFile(const QString& filename);
    bool closeFile();
    bool fileGood(){return m_fileGood;};

    const NUImage* GetImageData();
    const NUSensorsData* GetSensorData();
    const Localisation* GetLocalisationData();
    FieldObjects* GetObjectData();
    const GameInformation* GetGameInfo();
    const TeamInformation* GetTeamInfo();

    bool isNextFrameAvailable();
    bool isPreviousFrameAvailable();
    bool isFirstFrameAvailable();
    bool isLastFrameAvailable();
    bool isSetFrameAvailable();

    static std::vector<QFileInfo> FindValidFiles(const QDir& directory);

    std::vector<QFileInfo> AvailableLogFiles()const;
    QStringList AvailableData() const;

signals:

public slots:
    int nextFrame();
    int previousFrame();
    int firstFrame();
    int lastFrame();
    int setFrame(int frameNumber);

protected:
    std::vector<IndexedFileReader*> m_fileReaders;
    void setKnownDataTypes();
    StreamFileReader<NUImage> imageReader;
    StreamFileReader<NUSensorsData> sensorReader;
    StreamFileReader<NULocalisationSensors> locsensorReader;
    StreamFileReader<Localisation> locwmReader;
    StreamFileReader<FieldObjects> objectReader;
    StreamFileReader<LocWmFrame> locmframeReader;
    StreamFileReader<GameInformation> gameinfoReader;
    StreamFileReader<TeamInformation> teaminfoReader;
    QDir m_directory;
    QStringList m_knownDataTypes;
    QString m_extension;
    QString m_primaryData;
    bool m_fileGood;
    bool m_dataIsSynced;
    QStringList m_available_data;
    std::vector<QFileInfo> m_open_files;
    NUSensorsData m_tempSensors;
};

#endif // SPLITSTREAMFILEFORMATREADER_H
