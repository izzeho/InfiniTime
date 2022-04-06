#include "touchhandler/TouchHandler.h"

using namespace Pinetime::Controllers;

TouchHandler::TouchHandler(Drivers::Cst816S& touchPanel, Components::LittleVgl& lvgl) : touchPanel {touchPanel}, lvgl {lvgl} {
}

void TouchHandler::CancelTap() {
  if (info.touching) {
    isCancelled = true;
    lvgl.SetNewTouchPoint(-1, -1, true, false);
  }
}

Pinetime::Drivers::Cst816S::Gestures TouchHandler::GestureGet() {
  auto returnGesture = gesture;
  gesture = Drivers::Cst816S::Gestures::None;
  return returnGesture;
}

bool TouchHandler::GetNewTouchInfo() {
  info = touchPanel.GetTouchInfo();
  if (!info.isValid) return false;

  // REPORT configurations (P8b variants) of the fused (usually) Cst716
  // generate multiple "none" gesture events with info.touching == true during the physical gesture.
  // The last event is a e.g. "slide" event with info.touching == true.
  // gestureReleased state does not have to be computed manually, instead it occurs when event != "none".

  // GESTURE configurations (P8a variants) of the fused (usually) Cst716 generate no events during the physical gesture.
  // The only event is a e.g. "slide" event with info.touching == true.
  // gestureReleased state does not have to be computed manually, instead it occurs everytime.

  // DYNAMIC configurations (PineTime) are configured in reporting mode during initialisation.
  // Usually based on the Cst816s, they generate multiple e.g. "slide" gesture events with info.touching == true during the physical gesture.
  // The last of these e.g. "slide" events has info.touching == false.
  // gestureReleased state is computed manually by checking for the transition to info.touching == false.

  // Unfortunately, there is no way to reliably obtain which configuration is used at runtime.
  // In all cases, the event is bubbled up once the gesture is released.

  #if defined(DRIVER_TOUCH_REPORT)
    if (info.gesture != Pinetime::Drivers::Cst816S::Gestures::None) {
      gesture = info.gesture;
      info.touching = false;
    }
  #elif defined(DRIVER_TOUCH_GESTURE) 
    if (info.gesture != Pinetime::Drivers::Cst816S::Gestures::None) {
      gesture = info.gesture;
    }
  #elif defined(DRIVER_TOUCH_DYNAMIC)
    if (info.gesture != Pinetime::Drivers::Cst816S::Gestures::None) {
      if (gestureReleased) {
        if (info.gesture == Pinetime::Drivers::Cst816S::Gestures::SlideDown ||
            info.gesture == Pinetime::Drivers::Cst816S::Gestures::SlideLeft ||
            info.gesture == Pinetime::Drivers::Cst816S::Gestures::SlideUp ||
            info.gesture == Pinetime::Drivers::Cst816S::Gestures::SlideRight ||
            info.gesture == Pinetime::Drivers::Cst816S::Gestures::LongPress) {
          if (info.touching) {
            gesture = info.gesture;
            gestureReleased = false;
          }
        } else {
          gesture = info.gesture;
        }
      }
    }

    if (!info.touching) {
      gestureReleased = true;
    }
  #endif

  return true;
}

void TouchHandler::UpdateLvglTouchPoint() {
  if (info.touching) {
    #if defined(DRIVER_TOUCH_GESTURE)
      // GESTURE config only generates a single event / state change
      // so the LVGL wrapper is used to generate a successive release state update
      lvgl.SetNewTouchPoint(info.x, info.y, true, true);
    #else
      if (!isCancelled) {
        lvgl.SetNewTouchPoint(info.x, info.y, true, false);
      }
    #endif
  } else {
    if (isCancelled) {
      lvgl.SetNewTouchPoint(-1, -1, false, false);
      isCancelled = false;
    } else {
      lvgl.SetNewTouchPoint(info.x, info.y, false, false);
    }
  }
}
