/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../RoviSamplePlugin/src/SamplePlugin.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SamplePlugin_t {
    QByteArrayData data[44];
    char stringdata0[486];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SamplePlugin_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SamplePlugin_t qt_meta_stringdata_SamplePlugin = {
    {
QT_MOC_LITERAL(0, 0, 12), // "SamplePlugin"
QT_MOC_LITERAL(1, 13, 10), // "btnPressed"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 5), // "timer"
QT_MOC_LITERAL(4, 31, 20), // "stateChangedListener"
QT_MOC_LITERAL(5, 52, 21), // "rw::kinematics::State"
QT_MOC_LITERAL(6, 74, 5), // "state"
QT_MOC_LITERAL(7, 80, 17), // "getConfigurations"
QT_MOC_LITERAL(8, 98, 24), // "std::vector<rw::math::Q>"
QT_MOC_LITERAL(9, 123, 11), // "std::string"
QT_MOC_LITERAL(10, 135, 8), // "nameGoal"
QT_MOC_LITERAL(11, 144, 7), // "nameTcp"
QT_MOC_LITERAL(12, 152, 29), // "rw::models::SerialDevice::Ptr"
QT_MOC_LITERAL(13, 182, 5), // "robot"
QT_MOC_LITERAL(14, 188, 25), // "rw::models::WorkCell::Ptr"
QT_MOC_LITERAL(15, 214, 2), // "wc"
QT_MOC_LITERAL(16, 217, 15), // "checkCollisions"
QT_MOC_LITERAL(17, 233, 11), // "Device::Ptr"
QT_MOC_LITERAL(18, 245, 6), // "device"
QT_MOC_LITERAL(19, 252, 5), // "State"
QT_MOC_LITERAL(20, 258, 17), // "CollisionDetector"
QT_MOC_LITERAL(21, 276, 8), // "detector"
QT_MOC_LITERAL(22, 285, 1), // "Q"
QT_MOC_LITERAL(23, 287, 1), // "q"
QT_MOC_LITERAL(24, 289, 13), // "createPtPPath"
QT_MOC_LITERAL(25, 303, 20), // "rw::math::Vector3D<>"
QT_MOC_LITERAL(26, 324, 2), // "to"
QT_MOC_LITERAL(27, 327, 14), // "createPtPiPath"
QT_MOC_LITERAL(28, 342, 20), // "createPathRRTConnect"
QT_MOC_LITERAL(29, 363, 3), // "eps"
QT_MOC_LITERAL(30, 367, 4), // "TCMP"
QT_MOC_LITERAL(31, 372, 5), // "fRand"
QT_MOC_LITERAL(32, 378, 4), // "fMin"
QT_MOC_LITERAL(33, 383, 4), // "fMax"
QT_MOC_LITERAL(34, 388, 7), // "wrapMax"
QT_MOC_LITERAL(35, 396, 1), // "x"
QT_MOC_LITERAL(36, 398, 3), // "max"
QT_MOC_LITERAL(37, 402, 10), // "wrapMinMax"
QT_MOC_LITERAL(38, 413, 3), // "min"
QT_MOC_LITERAL(39, 417, 8), // "linePath"
QT_MOC_LITERAL(40, 426, 40), // "std::vector<rw::math::Vector3..."
QT_MOC_LITERAL(41, 467, 5), // "start"
QT_MOC_LITERAL(42, 473, 3), // "end"
QT_MOC_LITERAL(43, 477, 8) // "stepSize"

    },
    "SamplePlugin\0btnPressed\0\0timer\0"
    "stateChangedListener\0rw::kinematics::State\0"
    "state\0getConfigurations\0"
    "std::vector<rw::math::Q>\0std::string\0"
    "nameGoal\0nameTcp\0rw::models::SerialDevice::Ptr\0"
    "robot\0rw::models::WorkCell::Ptr\0wc\0"
    "checkCollisions\0Device::Ptr\0device\0"
    "State\0CollisionDetector\0detector\0Q\0q\0"
    "createPtPPath\0rw::math::Vector3D<>\0"
    "to\0createPtPiPath\0createPathRRTConnect\0"
    "eps\0TCMP\0fRand\0fMin\0fMax\0wrapMax\0x\0"
    "max\0wrapMinMax\0min\0linePath\0"
    "std::vector<rw::math::Vector3D<double> >\0"
    "start\0end\0stepSize"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SamplePlugin[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x08 /* Private */,
       3,    0,   80,    2, 0x08 /* Private */,
       4,    1,   81,    2, 0x08 /* Private */,
       7,    5,   84,    2, 0x08 /* Private */,
      16,    4,   95,    2, 0x08 /* Private */,
      24,    1,  104,    2, 0x08 /* Private */,
      27,    1,  107,    2, 0x08 /* Private */,
      28,    2,  110,    2, 0x08 /* Private */,
      30,    0,  115,    2, 0x08 /* Private */,
      31,    2,  116,    2, 0x08 /* Private */,
      34,    2,  121,    2, 0x08 /* Private */,
      37,    3,  126,    2, 0x08 /* Private */,
      39,    3,  133,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    0x80000000 | 8, 0x80000000 | 9, 0x80000000 | 9, 0x80000000 | 12, 0x80000000 | 14, 0x80000000 | 5,   10,   11,   13,   15,    6,
    QMetaType::Bool, 0x80000000 | 17, 0x80000000 | 19, 0x80000000 | 20, 0x80000000 | 22,   18,    6,   21,   23,
    QMetaType::Void, 0x80000000 | 25,   26,
    QMetaType::Void, 0x80000000 | 25,   26,
    QMetaType::Void, 0x80000000 | 25, QMetaType::Double,   26,   29,
    QMetaType::Void,
    QMetaType::Double, QMetaType::Double, QMetaType::Double,   32,   33,
    QMetaType::Double, QMetaType::Double, QMetaType::Double,   35,   36,
    QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   35,   38,   36,
    0x80000000 | 40, 0x80000000 | 25, 0x80000000 | 25, QMetaType::Double,   41,   42,   43,

       0        // eod
};

void SamplePlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SamplePlugin *_t = static_cast<SamplePlugin *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->btnPressed(); break;
        case 1: _t->timer(); break;
        case 2: _t->stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        case 3: { std::vector<rw::math::Q> _r = _t->getConfigurations((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const std::string(*)>(_a[2])),(*reinterpret_cast< rw::models::SerialDevice::Ptr(*)>(_a[3])),(*reinterpret_cast< rw::models::WorkCell::Ptr(*)>(_a[4])),(*reinterpret_cast< rw::kinematics::State(*)>(_a[5])));
            if (_a[0]) *reinterpret_cast< std::vector<rw::math::Q>*>(_a[0]) = std::move(_r); }  break;
        case 4: { bool _r = _t->checkCollisions((*reinterpret_cast< Device::Ptr(*)>(_a[1])),(*reinterpret_cast< const State(*)>(_a[2])),(*reinterpret_cast< const CollisionDetector(*)>(_a[3])),(*reinterpret_cast< const Q(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 5: _t->createPtPPath((*reinterpret_cast< rw::math::Vector3D<>(*)>(_a[1]))); break;
        case 6: _t->createPtPiPath((*reinterpret_cast< rw::math::Vector3D<>(*)>(_a[1]))); break;
        case 7: _t->createPathRRTConnect((*reinterpret_cast< rw::math::Vector3D<>(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 8: _t->TCMP(); break;
        case 9: { double _r = _t->fRand((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 10: { double _r = _t->wrapMax((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 11: { double _r = _t->wrapMinMax((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 12: { std::vector<rw::math::Vector3D<double> > _r = _t->linePath((*reinterpret_cast< rw::math::Vector3D<>(*)>(_a[1])),(*reinterpret_cast< rw::math::Vector3D<>(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< std::vector<rw::math::Vector3D<double> >*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject SamplePlugin::staticMetaObject = {
    { &rws::RobWorkStudioPlugin::staticMetaObject, qt_meta_stringdata_SamplePlugin.data,
      qt_meta_data_SamplePlugin,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(this);
    return rws::RobWorkStudioPlugin::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rws::RobWorkStudioPlugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}

QT_PLUGIN_METADATA_SECTION const uint qt_section_alignment_dummy = 42;

#ifdef QT_NO_DEBUG

QT_PLUGIN_METADATA_SECTION
static const unsigned char qt_pluginMetaData[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', ' ',
    'q',  'b',  'j',  's',  0x01, 0x00, 0x00, 0x00,
    0x10, 0x01, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00,
    0xfc, 0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x03, 0x00, 'I',  'I',  'D',  0x00, 0x00, 0x00,
    '*',  0x00, 'd',  'k',  '.',  's',  'd',  'u', 
    '.',  'm',  'i',  'p',  '.',  'R',  'o',  'b', 
    'w',  'o',  'r',  'k',  '.',  'R',  'o',  'b', 
    'W',  'o',  'r',  'k',  'S',  't',  'u',  'd', 
    'i',  'o',  'P',  'l',  'u',  'g',  'i',  'n', 
    '/',  '0',  '.',  '1',  0x9b, 0x0a, 0x00, 0x00,
    0x09, 0x00, 'c',  'l',  'a',  's',  's',  'N', 
    'a',  'm',  'e',  0x00, 0x0c, 0x00, 'S',  'a', 
    'm',  'p',  'l',  'e',  'P',  'l',  'u',  'g', 
    'i',  'n',  0x00, 0x00, 0xba, ' ',  0xa1, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
    0x05, 0x00, 'd',  'e',  'b',  'u',  'g',  0x00,
    0x15, 0x12, 0x00, 0x00, 0x08, 0x00, 'M',  'e', 
    't',  'a',  'D',  'a',  't',  'a',  0x00, 0x00,
    'l',  0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
    '`',  0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x04, 0x00, 'n',  'a',  'm',  'e',  0x00, 0x00,
    0x0b, 0x00, 'p',  'l',  'u',  'g',  'i',  'n', 
    'U',  'I',  'a',  'p',  'p',  0x00, 0x00, 0x00,
    0x1b, 0x07, 0x00, 0x00, 0x07, 0x00, 'v',  'e', 
    'r',  's',  'i',  'o',  'n',  0x00, 0x00, 0x00,
    0x05, 0x00, '1',  '.',  '0',  '.',  '0',  0x00,
    0x94, 0x0a, 0x00, 0x00, 0x0c, 0x00, 'd',  'e', 
    'p',  'e',  'n',  'd',  'e',  'n',  'c',  'i', 
    'e',  's',  0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    '@',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    '(',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 'D',  0x00, 0x00, 0x00,
    't',  0x00, 0x00, 0x00, 'd',  0x00, 0x00, 0x00
};

#else // QT_NO_DEBUG

QT_PLUGIN_METADATA_SECTION
static const unsigned char qt_pluginMetaData[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', ' ',
    'q',  'b',  'j',  's',  0x01, 0x00, 0x00, 0x00,
    0x10, 0x01, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00,
    0xfc, 0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x03, 0x00, 'I',  'I',  'D',  0x00, 0x00, 0x00,
    '*',  0x00, 'd',  'k',  '.',  's',  'd',  'u', 
    '.',  'm',  'i',  'p',  '.',  'R',  'o',  'b', 
    'w',  'o',  'r',  'k',  '.',  'R',  'o',  'b', 
    'W',  'o',  'r',  'k',  'S',  't',  'u',  'd', 
    'i',  'o',  'P',  'l',  'u',  'g',  'i',  'n', 
    '/',  '0',  '.',  '1',  0x95, 0x0a, 0x00, 0x00,
    0x08, 0x00, 'M',  'e',  't',  'a',  'D',  'a', 
    't',  'a',  0x00, 0x00, 'l',  0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, '`',  0x00, 0x00, 0x00,
    0x1b, 0x03, 0x00, 0x00, 0x04, 0x00, 'n',  'a', 
    'm',  'e',  0x00, 0x00, 0x0b, 0x00, 'p',  'l', 
    'u',  'g',  'i',  'n',  'U',  'I',  'a',  'p', 
    'p',  0x00, 0x00, 0x00, 0x1b, 0x07, 0x00, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x05, 0x00, '1',  '.', 
    '0',  '.',  '0',  0x00, 0x94, 0x0a, 0x00, 0x00,
    0x0c, 0x00, 'd',  'e',  'p',  'e',  'n',  'd', 
    'e',  'n',  'c',  'i',  'e',  's',  0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, '@',  0x00, 0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, '(',  0x00, 0x00, 0x00,
    0x1b, 0x1a, 0x00, 0x00, 0x09, 0x00, 'c',  'l', 
    'a',  's',  's',  'N',  'a',  'm',  'e',  0x00,
    0x0c, 0x00, 'S',  'a',  'm',  'p',  'l',  'e', 
    'P',  'l',  'u',  'g',  'i',  'n',  0x00, 0x00,
    '1',  0x00, 0x00, 0x00, 0x05, 0x00, 'd',  'e', 
    'b',  'u',  'g',  0x00, 0xba, ' ',  0xa1, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    'D',  0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00,
    0xe0, 0x00, 0x00, 0x00, 0xec, 0x00, 0x00, 0x00
};
#endif // QT_NO_DEBUG

QT_MOC_EXPORT_PLUGIN(SamplePlugin, SamplePlugin)

QT_WARNING_POP
QT_END_MOC_NAMESPACE
