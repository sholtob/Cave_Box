/*
    The code in this file uses open source libraries provided by thecoderscorner

    DO NOT EDIT THIS FILE, IT WILL BE GENERATED EVERY TIME YOU USE THE UI DESIGNER
    INSTEAD EITHER PUT CODE IN YOUR SKETCH OR CREATE ANOTHER SOURCE FILE.

    All the variables you may need access to are marked extern in this file for easy
    use elsewhere.
 */

#include <tcMenu.h>
#include "shroom_room_oled_tc_menu_menu.h"
#include "ThemeMonoBordered.h"

// Global variable declarations
const PROGMEM  ConnectorLocalInfo applicationInfo = { "CAVE BOX", "1a525a77-f493-4b28-b422-b99265950d76" };
ArduinoEEPROMAbstraction glArduinoEeprom(&EEPROM);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C gfx(U8G2_R2, U8X8_PIN_NONE, 22, 21);
U8g2Drawable gfxDrawable(&gfx);
GraphicsDeviceRenderer renderer(30, applicationInfo.name, &gfxDrawable);

// Global Menu Item declarations
const PROGMEM AnyMenuInfo minfoVersion = { "Version", 27, 0xffff, 0, firmwareVersionCallback };
ActionMenuItem menuVersion(&minfoVersion, nullptr, INFO_LOCATION_PGM);
const PROGMEM AnyMenuInfo minfoFactoryReset = { "Factory Reset", 13, 0xffff, 0, factoryReset };
ActionMenuItem menuFactoryReset(&minfoFactoryReset, &menuVersion, INFO_LOCATION_PGM);
const PROGMEM AnyMenuInfo minfoPrintSettings = { "Print Settings", 26, 0xffff, 0, PrintSettingsCallback };
ActionMenuItem menuPrintSettings(&minfoPrintSettings, &menuFactoryReset, INFO_LOCATION_PGM);
const PROGMEM AnalogMenuInfo minfoDValue = { "D Value", 16, 14, 2000, DValueCallback, 0, 1, "" };
AnalogMenuItem menuDValue(&minfoDValue, 0, nullptr, INFO_LOCATION_PGM);
const PROGMEM AnalogMenuInfo minfoIValue = { "I Value", 15, 12, 10000, IValueCallback, 0, 1000, "" };
AnalogMenuItem menuIValue(&minfoIValue, 0, &menuDValue, INFO_LOCATION_PGM);
const PROGMEM AnalogMenuInfo minfoPValue = { "P Value", 14, 10, 2000, PValueCallback, 0, 1, "" };
AnalogMenuItem menuPValue(&minfoPValue, 0, &menuIValue, INFO_LOCATION_PGM);
const PROGMEM SubMenuInfo minfoEditPIDVariables = { "Edit PID Variables", 9, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackEditPIDVariables(&minfoEditPIDVariables, &menuPValue, INFO_LOCATION_PGM);
SubMenuItem menuEditPIDVariables(&minfoEditPIDVariables, &menuBackEditPIDVariables, &menuPrintSettings, INFO_LOCATION_PGM);
const PROGMEM AnalogMenuInfo minfoMaxRH = { "Max RH", 25, 24, 100, MaxRHCallback, 0, 1, " %" };
AnalogMenuItem menuMaxRH(&minfoMaxRH, 98, &menuEditPIDVariables, INFO_LOCATION_PGM);
const PROGMEM AnalogMenuInfo minfoFAEOnTime = { "FAE On Time", 20, 17, 60, FAEOnTimeCallBack, 0, 1, " min" };
AnalogMenuItem menuFAEOnTime(&minfoFAEOnTime, 1, &menuMaxRH, INFO_LOCATION_PGM);
const PROGMEM AnalogMenuInfo minfoLightOnTime = { "Light On Time", 22, 19, 24, lightOntimeCallback, 0, 1, " hrs" };
AnalogMenuItem menuLightOnTime(&minfoLightOnTime, 24, &menuFAEOnTime, INFO_LOCATION_PGM);
const PROGMEM BooleanMenuInfo minfoLEDShowRH = { "LED Show RH", 23, 21, 1, LEDShowRHCallback, NAMING_TRUE_FALSE };
BooleanMenuItem menuLEDShowRH(&minfoLEDShowRH, true, &menuLightOnTime, INFO_LOCATION_PGM);
const PROGMEM BooleanMenuInfo minfoenableFan = { "Enable Fan", 18, 16, 1, NO_CALLBACK, NAMING_ON_OFF };
BooleanMenuItem menuenableFan(&minfoenableFan, true, &menuLEDShowRH, INFO_LOCATION_PGM);
const PROGMEM SubMenuInfo minfoAdvanced = { "Advanced", 19, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackAdvanced(&minfoAdvanced, &menuenableFan, INFO_LOCATION_PGM);
SubMenuItem menuAdvanced(&minfoAdvanced, &menuBackAdvanced, nullptr, INFO_LOCATION_PGM);
const PROGMEM AnyMenuInfo minfoCalibrateFAE = { "Calibrate FAE", 21, 0xffff, 0, CalibrateFAECallback };
ActionMenuItem menuCalibrateFAE(&minfoCalibrateFAE, &menuAdvanced, INFO_LOCATION_PGM);
const PROGMEM AnyMenuInfo minfoSaveSettings = { "Save Settings", 17, 0xffff, 0, SaveSettingsCallback };
ActionMenuItem menuSaveSettings(&minfoSaveSettings, &menuCalibrateFAE, INFO_LOCATION_PGM);
const PROGMEM AnalogMenuInfo minfoFAEPerHour = { "FAE Per Hr", 6, 8, 60, FAEChangedCallback, 0, 1, "" };
AnalogMenuItem menuFAEPerHour(&minfoFAEPerHour, 5, &menuSaveSettings, INFO_LOCATION_PGM);
const PROGMEM AnalogMenuInfo minfoRHSetPoint = { "Set Humidity", 5, 6, 95, RHChangedCallback, 0, 1, "%" };
AnalogMenuItem menuRHSetPoint(&minfoRHSetPoint, 0, &menuFAEPerHour, INFO_LOCATION_PGM);
const PROGMEM SubMenuInfo minfoOptions = { "Settings", 4, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackOptions(&minfoOptions, &menuRHSetPoint, INFO_LOCATION_PGM);
SubMenuItem menuOptions(&minfoOptions, &menuBackOptions, nullptr, INFO_LOCATION_PGM);
const char enumStrPresets_0[] PROGMEM = "Pin";
const char enumStrPresets_1[] PROGMEM = "Fruit";
const char enumStrPresets_2[] PROGMEM = "Oyster";
const char enumStrPresets_3[] PROGMEM = "Lions Mane";
const char enumStrPresets_4[] PROGMEM = "Shiitake";
const char enumStrPresets_5[] PROGMEM = "Reishi";
const char enumStrPresets_6[] PROGMEM = "Custom1";
const char enumStrPresets_7[] PROGMEM = "Custom2";
const char enumStrPresets_8[] PROGMEM = "Custom3";
const char enumStrPresets_9[] PROGMEM = "Custom4";
const char enumStrPresets_10[] PROGMEM = "Custom5";
const char enumStrPresets_11[] PROGMEM = "Change6";
const char* const enumStrPresets[] PROGMEM  = { enumStrPresets_0, enumStrPresets_1, enumStrPresets_2, enumStrPresets_3, enumStrPresets_4, enumStrPresets_5, enumStrPresets_6, enumStrPresets_7, enumStrPresets_8, enumStrPresets_9, enumStrPresets_10, enumStrPresets_11 };
const PROGMEM EnumMenuInfo minfoPresets = { "Presets", 24, 22, 11, PresetsCallback, enumStrPresets };
EnumMenuItem menuPresets(&minfoPresets, 1, &menuOptions, INFO_LOCATION_PGM);
const PROGMEM BooleanMenuInfo minfoControllerBool = { "Controller", 8, 4, 1, controllerEnableCallback, NAMING_ON_OFF };
BooleanMenuItem menuControllerBool(&minfoControllerBool, true, &menuPresets, INFO_LOCATION_PGM);
const PROGMEM BooleanMenuInfo minfoEnableLED = { "LED", 3, 5, 1, enableLEDCallback, NAMING_ON_OFF };
BooleanMenuItem menuEnableLED(&minfoEnableLED, true, &menuControllerBool, INFO_LOCATION_PGM);
const PROGMEM AnyMenuInfo minfoTString = { "Temperature", 2, 0xffff, 0, NO_CALLBACK };
TextMenuItem menuTString(&minfoTString, "-1", 11, &menuEnableLED, INFO_LOCATION_PGM);
const PROGMEM AnyMenuInfo minfoRHString = { "Humidity", 1, 0xffff, 0, NO_CALLBACK };
TextMenuItem menuRHString(&minfoRHString, "-1", 11, &menuTString, INFO_LOCATION_PGM);

void setupMenu() {
    // First we set up eeprom and authentication (if needed).
    setSizeBasedEEPROMStorageEnabled(true);
    menuMgr.setEepromRef(&glArduinoEeprom);
    // Now add any readonly, non-remote and visible flags.
    menuRHString.setReadOnly(true);
    menuTString.setReadOnly(true);

    // Code generated by plugins.
    gfx.begin();
    renderer.setUpdatesPerSecond(10);
    switches.init(internalDigitalIo(), SWITCHES_NO_POLLING, true);
    menuMgr.initForEncoder(&renderer, &menuRHString, 23, 19, 25);
    renderer.setTitleMode(BaseGraphicalRenderer::TITLE_FIRST_ROW);
    renderer.setUseSliderForAnalog(false);
    installMonoBorderedTheme(renderer, MenuFontDef(nullptr, 1), MenuFontDef(nullptr, 1), true);
}

