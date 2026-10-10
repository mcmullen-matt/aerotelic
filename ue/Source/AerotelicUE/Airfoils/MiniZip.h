#pragma once

#include "CoreMinimal.h"

// Just enough of the zip format to unpack the UIUC coordinate archive
// (stored or deflated entries, no encryption, no zip64): the engine has
// zlib but no archive reader available to a game module.
struct FMiniZipEntry
{
	FString Name;
	uint16 Method = 0;        // 0 = stored, 8 = deflate
	uint32 CompressedSize = 0;
	uint32 UncompressedSize = 0;
	uint32 LocalHeaderOffset = 0;
};

// Reads the central directory. False (with a reason) if the data is not a
// zip archive this reader understands.
bool MiniZipList(const TArray<uint8>& Archive, TArray<FMiniZipEntry>& OutEntries, FString& OutError);

// Unpacks one entry.
bool MiniZipExtract(const TArray<uint8>& Archive, const FMiniZipEntry& Entry, TArray<uint8>& OutData, FString& OutError);
