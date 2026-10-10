#include "MiniZip.h"

#include "Misc/Compression.h"

namespace
{
	uint16 Read16(const TArray<uint8>& D, int64 At) { return (uint16)D[At] | ((uint16)D[At + 1] << 8); }
	uint32 Read32(const TArray<uint8>& D, int64 At) { return (uint32)D[At] | ((uint32)D[At + 1] << 8) | ((uint32)D[At + 2] << 16) | ((uint32)D[At + 3] << 24); }

	constexpr uint32 kEndOfCentralDir = 0x06054b50;
	constexpr uint32 kCentralFileHeader = 0x02014b50;
	constexpr uint32 kLocalFileHeader = 0x04034b50;
}

bool MiniZipList(const TArray<uint8>& Archive, TArray<FMiniZipEntry>& OutEntries, FString& OutError)
{
	OutEntries.Reset();
	const int64 Size = Archive.Num();
	if (Size < 22)
	{
		OutError = TEXT("not a zip archive (too short)");
		return false;
	}
	// The end-of-central-directory record is in the last 64 KB + 22 bytes
	// (it ends with a comment of up to 65535 bytes).
	int64 Eocd = -1;
	const int64 Lowest = FMath::Max<int64>(0, Size - 22 - 65535);
	for (int64 At = Size - 22; At >= Lowest; --At)
	{
		if (Read32(Archive, At) == kEndOfCentralDir)
		{
			Eocd = At;
			break;
		}
	}
	if (Eocd < 0)
	{
		OutError = TEXT("not a zip archive (no end-of-central-directory record)");
		return false;
	}
	const uint32 Count = Read16(Archive, Eocd + 10);
	const uint32 CdSize = Read32(Archive, Eocd + 12);
	const uint32 CdOffset = Read32(Archive, Eocd + 16);
	if ((int64)CdOffset + (int64)CdSize > Size)
	{
		OutError = TEXT("zip central directory lies outside the file");
		return false;
	}
	int64 At = CdOffset;
	for (uint32 i = 0; i < Count; ++i)
	{
		if (At + 46 > Size || Read32(Archive, At) != kCentralFileHeader)
		{
			OutError = TEXT("zip central directory is damaged");
			return false;
		}
		FMiniZipEntry E;
		E.Method = Read16(Archive, At + 10);
		E.CompressedSize = Read32(Archive, At + 20);
		E.UncompressedSize = Read32(Archive, At + 24);
		const uint16 NameLen = Read16(Archive, At + 28);
		const uint16 ExtraLen = Read16(Archive, At + 30);
		const uint16 CommentLen = Read16(Archive, At + 32);
		E.LocalHeaderOffset = Read32(Archive, At + 42);
		if (At + 46 + NameLen > Size)
		{
			OutError = TEXT("zip central directory is damaged");
			return false;
		}
		// Names are UTF-8 (or CP437 for old archives; the UIUC names are ASCII).
		FUTF8ToTCHAR Conv((const ANSICHAR*)&Archive[At + 46], NameLen);
		E.Name = FString(Conv.Length(), Conv.Get());
		OutEntries.Add(E);
		At += 46 + NameLen + ExtraLen + CommentLen;
	}
	return true;
}

bool MiniZipExtract(const TArray<uint8>& Archive, const FMiniZipEntry& Entry, TArray<uint8>& OutData, FString& OutError)
{
	const int64 Size = Archive.Num();
	const int64 At = Entry.LocalHeaderOffset;
	if (At + 30 > Size || Read32(Archive, At) != kLocalFileHeader)
	{
		OutError = FString::Printf(TEXT("zip entry '%s' has a damaged local header"), *Entry.Name);
		return false;
	}
	const uint16 NameLen = Read16(Archive, At + 26);
	const uint16 ExtraLen = Read16(Archive, At + 28);
	const int64 DataAt = At + 30 + NameLen + ExtraLen;
	if (DataAt + (int64)Entry.CompressedSize > Size)
	{
		OutError = FString::Printf(TEXT("zip entry '%s' lies outside the file"), *Entry.Name);
		return false;
	}
	OutData.SetNumUninitialized(Entry.UncompressedSize);
	if (Entry.Method == 0)
	{
		if (Entry.CompressedSize != Entry.UncompressedSize)
		{
			OutError = FString::Printf(TEXT("zip entry '%s': stored sizes disagree"), *Entry.Name);
			return false;
		}
		FMemory::Memcpy(OutData.GetData(), &Archive[DataAt], Entry.UncompressedSize);
		return true;
	}
	if (Entry.Method == 8)
	{
		if (Entry.UncompressedSize == 0)
		{
			return true;
		}
		// Raw deflate: a negative window size tells zlib there is no zlib
		// header (the engine passes the value straight to inflateInit2).
		const uintptr_t RawDeflate = (uintptr_t)(intptr_t)(-15);
		if (!FCompression::UncompressMemory(NAME_Zlib, OutData.GetData(), Entry.UncompressedSize, &Archive[DataAt], Entry.CompressedSize, COMPRESS_NoFlags, RawDeflate))
		{
			OutError = FString::Printf(TEXT("zip entry '%s' did not inflate"), *Entry.Name);
			return false;
		}
		return true;
	}
	OutError = FString::Printf(TEXT("zip entry '%s' uses an unsupported compression method (%d)"), *Entry.Name, Entry.Method);
	return false;
}
