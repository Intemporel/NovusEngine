#pragma once
#include "FileFormat/Novus/FileHeader.h"

#include <Base/Types.h>
#include <Base/Platform.h>

#include <vector>

class Bytebuffer;
namespace CLIENTDB
{
	PRAGMA_NO_PADDING_START;
	template <class T>
	struct ClientDB
	{
	public:
		static const u32 CURRENT_VERSION = 1;

		struct Flags
		{
			u32 unused : 32;
		};

	public:
		FileHeader header = FileHeader(FileHeader::Type::ClientDB, CURRENT_VERSION);

		Flags flags = { };
		std::vector<T> data = { };

	public:
		bool Save(std::string& path)
		{
            // Create a file
            std::ofstream output(path, std::ofstream::out | std::ofstream::binary);
            if (!output)
            {
                DebugHandler::PrintError("Failed to create Terrain Chunk file. Check admin permissions");
                return false;
            }

            // Write the Chunk to file
            output.write(reinterpret_cast<char const*>(&header), sizeof(FileHeader));
            output.write(reinterpret_cast<char const*>(&flags), sizeof(u32));

            u32 numElements = static_cast<u32>(data.size());
            output.write(reinterpret_cast<char const*>(&numElements), sizeof(u32)); // Write number of elements

            if (numElements > 0)
            {
                output.write(reinterpret_cast<char const*>(data.data()), data * sizeof(T)); // Write elements
            }

            output.close();

            return true;
		}

		bool Write(std::shared_ptr<Bytebuffer>& buffer)
		{
            if (!buffer->Put(header))
                return false;

            if (!buffer->PutU32(flags))
                return false;

            // Read Elements
            {
                u32 numElements = static_cast<u32>(data.size());
                if (!buffer->PutU32(numElements))
                    return false;

                if (numElements)
                {
                    if (!buffer->PutBytes(reinterpret_cast<u8*>(&data[0]), numElements * sizeof(T)))
                        return false;
                }
            }

            return true;
		}

		static bool Read(std::shared_ptr<Bytebuffer>& buffer, ClientDB& out)
		{
			out = { };

			if (!buffer->Get(out.header))
				return false;

			if (!buffer->Get(out.flags))
				return false;

            // Read Elements
            {
                u32 numElements = 0;
                if (!buffer->GetU32(numElements))
                    return false;

                if (numElements)
                {
                    out.mapObjectPlacements.resize(numElements);
                    if (!buffer->GetBytes(reinterpret_cast<u8*>(&out.data[0]), numElements * sizeof(T)))
                        return false;
                }
            }

            return true;
		}
	};
	PRAGMA_NO_PADDING_END;
}