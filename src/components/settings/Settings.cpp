#include "components/settings/Settings.h"
#include <cstdlib>
#include <cstring>

using namespace Pinetime::Controllers;

Settings::Settings(Pinetime::Controllers::FS& fs, Pinetime::Drivers::AccelerationSensor& acc) : fs {fs}, acc {acc} {
}

void Settings::Init() {

  // Load default settings from Flash
  LoadSettingsFromFile();
}

void Settings::SaveSettings() {

  // verify if is necessary to save
  if (settingsChanged) {
    SaveSettingsToFile();
  }
  settingsChanged = false;
}

void Settings::LoadSettingsFromFile() {
  SettingsData bufferSettings;
  lfs_file_t settingsFile;

  if ( fs.FileOpen(&settingsFile, "/settings.dat", LFS_O_RDONLY) != LFS_ERR_OK) {
    return;
  }
  fs.FileRead(&settingsFile, reinterpret_cast<uint8_t*>(&bufferSettings), sizeof(settings));
  fs.FileClose(&settingsFile);
  if ( bufferSettings.version == settingsVersion ) {
    settings = bufferSettings;
  }

  uploadWakeUpMode();
}

void Settings::SaveSettingsToFile() {
  lfs_file_t settingsFile;

  if ( fs.FileOpen(&settingsFile, "/settings.dat", LFS_O_WRONLY | LFS_O_CREAT) != LFS_ERR_OK) {
    return;
  }
  fs.FileWrite(&settingsFile, reinterpret_cast<uint8_t*>(&settings), sizeof(settings));
  fs.FileClose(&settingsFile);
}

void Settings::uploadWakeUpMode() {
  // Setup the motion sensor interrupt configuration
  if(isWakeUpModeOn(Settings::WakeUpMode::DoubleTap)) {
    acc.SetMotion(Drivers::MotionEvents::DoubleTap);
  } else if(isWakeUpModeOn(Settings::WakeUpMode::SingleTap)) {
    acc.SetMotion(Drivers::MotionEvents::SingleTap);
  } else {
    acc.SetMotion(Drivers::MotionEvents::None);
  }
}
