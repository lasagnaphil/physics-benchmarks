# Locate the PhysX SDK

if (NOT DEFINED PHYSX_HOME)
    set(PHYSX_HOME $ENV{PHYSX_HOME})
endif()
if (NOT DEFINED PXSHARED_HOME)
    set(PXSHARED_HOME $ENV{PXSHARED_HOME})
endif()

set(PHYSX_HOME /home/lasagnaphil/packages/PhysX/physx/install/linux/PhysX)
set(PXSHARED_HOME /home/lasagnaphil/packages/PhysX/physx/install/linux/PxShared)

set(PhysX_INCLUDE_DIR ${PHYSX_HOME}/include)
set(PxShared_INCLUDE_DIR ${PXSHARED_HOME}/include)

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(LIBFOLDERSUFFIX "64")
    set(PHYSXPREFIX "_x64")
else()
    set(LIBFOLDERSUFFIX "32")
endif()

if (NOT PhysX_LIBRARY_DIR)
    set(PhysX_LIBRARY_DIR ${PHYSX_HOME}/bin/linux.clang)
endif()

set(PhysX_LIBRARIES_DEBUG "")
set(PhysX_LIBRARIES_PROFILE "")
set(PhysX_LIBRARIES_CHECKED "")
set(PhysX_LIBRARIES_RELEASE "")

macro(ADD_PHYSX_LIBRARY LIBNAME LIBTYPE)
    string(TOUPPER ${LIBNAME} LIBNAME_CAPS)
    string(TOUPPER ${LIBTYPE} LIBTYPE_CAPS)
    find_library(PhysX_LIBRARY_${LIBNAME_CAPS}_${LIBTYPE_CAPS}
            NAMES libPhysX${LIBNAME}_static_${LIBFOLDERSUFFIX}.a
            PATHS ${PhysX_LIBRARY_DIR}/${LIBTYPE})

    list(APPEND PhysX_LIBRARIES_${LIBTYPE_CAPS} ${PhysX_LIBRARY_${LIBNAME_CAPS}_${LIBTYPE_CAPS}})
endmacro()

foreach(LIBTYPE debug profile checked release)
    string(TOUPPER ${LIBTYPE} LIBTYPE_CAPS)
    find_library(PhysX_LIBRARY_BASE_${LIBTYPE_CAPS}
            NAMES libPhysX_static_${LIBFOLDERSUFFIX}.a
            PATHS ${PhysX_LIBRARY_DIR}/${LIBTYPE})
    list(APPEND PhysX_LIBRARIES_${LIBTYPE_CAPS} ${PhysX_LIBRARY_BASE_${LIBTYPE_CAPS}})

    ADD_PHYSX_LIBRARY(CharacterKinematic ${LIBTYPE})
    ADD_PHYSX_LIBRARY(Common ${LIBTYPE})
    ADD_PHYSX_LIBRARY(Cooking ${LIBTYPE})
    ADD_PHYSX_LIBRARY(Extensions ${LIBTYPE})
    ADD_PHYSX_LIBRARY(Foundation ${LIBTYPE})
    ADD_PHYSX_LIBRARY(PvdSDK ${LIBTYPE})
    ADD_PHYSX_LIBRARY(Vehicle ${LIBTYPE})
endforeach()

set(PhysX_LIBRARIES debug ${PhysX_LIBRARIES_DEBUG})

if (PhysX_PROFILE)
    set(PhysX_LIBRARIES ${PhysX_LIBRARIES} optimized ${PhysX_LIBRARIES_PROFILE})
elseif (PhysX_CHECKED)
    set(PhysX_LIBRARIES ${PhysX_LIBRARIES} optimized ${PhysX_LIBRARIES_CHECKED})
else ()
    set(PhysX_LIBRARIES ${PhysX_LIBRARIES} optimized ${PhysX_LIBRARIES_RELEASE})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PhysX  DEFAULT_MSG  PhysX_INCLUDE_DIR PxShared_INCLUDE_DIR PhysX_LIBRARIES)

# mark_as_advanced(PhysX_INCLUDE_DIR PxShared_INCLUDE_DIR PhysX_LIBRARY_DIR PhysX_LIBRARY_DEBUG PhysX_LIBRARY_RELEASE PhysX_LIBRARY_PROFILE PhysX_LIBRARY_CHECKED)