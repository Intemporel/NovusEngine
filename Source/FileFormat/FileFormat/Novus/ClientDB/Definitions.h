#pragma once
#include <Base/Types.h>

namespace DB::Client::Definitions
{
    struct Map
    {
    public:
        u32 id;
        u32 name;
        u32 internalName;
        u32 instanceType;
        u32 flags;
        u32 expansion;
        u32 maxPlayers;
    };

    struct LiquidObject
    {
    public:
        u32 id;
        u16 liquidTypeID;
        f32 flowDirection;
        f32 flowSpeed;
        u8 fishable;
        u8 reflection;
    };

    struct LiquidType
    {
    public:
        u32 id;
        u32 name;
        u32 textures[6];
        u16 flags;
        u8 soundBank;
        u32 soundID;
        u32 spellID;
        f32 maxDarkenDepth;
        f32 fogDarkenIntensity;
        f32 ambDarkenIntensity;
        f32 dirDarkenIntensity;
        u16 lightID;
        f32 particleScale;
        u8 particleMovement;
        u8 particleTexSlots;
        u8 materialID;
        u32 minimapStaticCol;
        u32 frameCountTextures[6];
        u32 colors[2];
        f32 unkFloats[16];
        u32 unkInts[4];
        u32 coefficients[4];
    };

    struct LiquidMaterial
    {
    public:
        u32 id;
        u8 flags;
        u8 liquidVertexFormat;
    };

    struct CinematicCamera
    {
    public:
        u32 id;
        vec3 endPosition;
        u32 soundID;
        f32 rotation;
        u32 cameraPath;
    };

    struct CinematicSequence
    {
    public:
        u32 id;
        u32 soundID;
        u32 cameraIDs[8];
    };

    struct Cinematic
    {
    public:
        u32 id                  = 0;
        u32 musicSoundID        = 0;        // index in SoundEntries (old)
        u32 flags               = 0;
        f32 rotation            = 0.0f;
        vec3 position           = vec3( 0.0f );
        struct Sequence
        {
            u32 timestamp       = 0;
            u32 musicSoundID    = 0;        // index in SoundEntries (old)
            u32 positionSpline  = 0;        // index in SplineData
            u32 targetSpline    = 0;        // index in SplineData
            u32 rollSpline      = 0;        // index in SplineData
            u32 fovSpline       = 0;        // index in SplineData
        } Sequences[8];
    };

    struct SplineData
    {
    public:
        u32 id          = 0;
        u32 path        = 0;
    };
}