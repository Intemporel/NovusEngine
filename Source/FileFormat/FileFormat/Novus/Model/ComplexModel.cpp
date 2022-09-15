#include "ComplexModel.h"
#include "FileFormat/Warcraft/Shared.h"
#include "FileFormat/Warcraft/M2/M2.h"

#include <Base/Memory/Bytebuffer.h>
#include <Base/Util/DebugHandler.h>

#include <fstream>

using namespace M2;

namespace Model
{
	bool ComplexModel::Save(const std::string& path)
	{
		// Create a file
		std::ofstream output(path, std::ofstream::out | std::ofstream::binary);
		if (!output)
		{
			DebugHandler::PrintError("Failed to create CModel file. Check admin permissions");
			return false;
		}

		// Write Header
		{
			output.write(reinterpret_cast<const char*>(&header), sizeof(FileHeader));
			output.write(reinterpret_cast<const char*>(&flags), sizeof(ComplexModel::Flags));
		}

		// Write Sequences
		{
			u32 numElements = static_cast<u32>(sequences.size());
			output.write(reinterpret_cast<const char*>(&numElements), sizeof(u32));

			if (numElements > 0)
			{
				output.write(reinterpret_cast<const char*>(&sequences[0]), numElements * sizeof(ComplexModel::AnimationSequence));
			}
		}

		// Write Bones
		{
			u32 numElements = static_cast<u32>(bones.size());
			output.write(reinterpret_cast<const char*>(&numElements), sizeof(u32));

			if (numElements > 0)
			{
				for (const ComplexModel::Bone& bone : bones)
				{
					output.write(reinterpret_cast<const char*>(&bone.primaryBoneIndex), sizeof(i32));
					output.write(reinterpret_cast<const char*>(&bone.flags), sizeof(u32));

					output.write(reinterpret_cast<const char*>(&bone.parentBoneID), sizeof(i16));
					output.write(reinterpret_cast<const char*>(&bone.submeshID), sizeof(u16));

					bone.translation.Serialize(output);
					bone.rotation.Serialize(output);
					bone.scale.Serialize(output);

					output.write(reinterpret_cast<const char*>(&bone.pivot), sizeof(vec3));
				}
			}
		}

		// Write Vertices
		{
			u32 numVertices = static_cast<u32>(vertices.size());
			output.write(reinterpret_cast<const char*>(&numVertices), sizeof(u32));

			if (numVertices > 0)
			{
				output.write(reinterpret_cast<const char*>(&vertices[0]), numVertices * sizeof(ComplexModel::Vertex));
			}
		}

		// Write Textures
		{
			u32 numTextures = static_cast<u32>(textures.size());
			output.write(reinterpret_cast<const char*>(&numTextures), sizeof(u32));

			if (numTextures > 0)
			{
				for (u32 i = 0; i < textures.size(); i++)
				{
					const ComplexModel::Texture& texture = textures[i];

					output.write(reinterpret_cast<const char*>(&texture.type), sizeof(ComplexModel::Texture::Type));
					output.write(reinterpret_cast<const char*>(&texture.flags), sizeof(ComplexModel::Texture::Flags));
					output.write(reinterpret_cast<const char*>(&texture.textureHash), sizeof(u32));
				}
			}
		}

		// Write Materials
		{
			u32 numMaterials = static_cast<u32>(materials.size());
			output.write(reinterpret_cast<const char*>(&numMaterials), sizeof(u32));

			if (numMaterials > 0)
			{
				output.write(reinterpret_cast<const char*>(&materials[0]), numMaterials * sizeof(ComplexModel::Material));
			}
		}

		// Write Texture Transforms
		{
			u32 numTextureTransforms = static_cast<u32>(textureTransforms.size());
			output.write(reinterpret_cast<const char*>(&numTextureTransforms), sizeof(u32));

			if (numTextureTransforms > 0)
			{
				for (u32 i = 0; i < textureTransforms.size(); i++)
				{
					const ComplexModel::TextureTransform& textureTransform = textureTransforms[i];

					textureTransform.translation.Serialize(output);
					textureTransform.rotation.Serialize(output);
					textureTransform.scale.Serialize(output);
				}
			}
		}

		// Write Texture Index Lookup Table
		{
			u32 numTextureIndexLookupIDs = static_cast<u32>(textureIndexLookupTable.size());
			output.write(reinterpret_cast<const char*>(&numTextureIndexLookupIDs), sizeof(u32));

			if (numTextureIndexLookupIDs > 0)
			{
				output.write(reinterpret_cast<const char*>(&textureIndexLookupTable[0]), numTextureIndexLookupIDs * sizeof(u16));
			}
		}

		// Write Texture Unit Lookup Table
		{
			u32 numTextureUnitLookupIDs = static_cast<u32>(textureUnitLookupTable.size());
			output.write(reinterpret_cast<const char*>(&numTextureUnitLookupIDs), sizeof(u32));

			if (numTextureUnitLookupIDs > 0)
			{
				output.write(reinterpret_cast<const char*>(&textureUnitLookupTable[0]), numTextureUnitLookupIDs * sizeof(u16));
			}
		}

		// Write Texture Transparency Lookup Table
		{
			u32 numTextureTransparencyLookupIDs = static_cast<u32>(textureTransparencyLookupTable.size());
			output.write(reinterpret_cast<const char*>(&numTextureTransparencyLookupIDs), sizeof(u32));

			if (numTextureTransparencyLookupIDs > 0)
			{
				output.write(reinterpret_cast<const char*>(&textureTransparencyLookupTable[0]), numTextureTransparencyLookupIDs * sizeof(u16));
			}
		}

		// Write Texture Transform Lookup Table
		{
			u32 numTextureTransformLookupIDs = static_cast<u32>(textureTransformLookupTable.size());
			output.write(reinterpret_cast<const char*>(&numTextureTransformLookupIDs), sizeof(u32));

			if (numTextureTransformLookupIDs > 0)
			{
				output.write(reinterpret_cast<const char*>(&textureTransformLookupTable[0]), numTextureTransformLookupIDs * sizeof(u16));
			}
		}

		// Write Texture Combiner Combos
		{
			u32 numTextureCombinerCombos = static_cast<u32>(textureCombinerCombos.size());
			output.write(reinterpret_cast<const char*>(&numTextureCombinerCombos), sizeof(u32));

			if (numTextureCombinerCombos > 0)
			{
				output.write(reinterpret_cast<const char*>(&textureCombinerCombos[0]), numTextureCombinerCombos * sizeof(u16));
			}
		}

		// Write Collision Data
		{
			// Collision Vertex Positions
			{
				u32 numCollisionVertexPositions = static_cast<u32>(collisionVertexPositions.size());
				output.write(reinterpret_cast<const char*>(&numCollisionVertexPositions), sizeof(u32));

				if (numCollisionVertexPositions > 0)
				{
					output.write(reinterpret_cast<const char*>(&collisionVertexPositions[0]), numCollisionVertexPositions * sizeof(vec3));
				}
			}

			// Collision Indices
			{
				u32 numCollisionIndices = static_cast<u32>(collisionIndices.size());
				output.write(reinterpret_cast<const char*>(&numCollisionIndices), sizeof(u32));

				if (numCollisionIndices > 0)
				{
					output.write(reinterpret_cast<const char*>(&collisionIndices[0]), numCollisionIndices * sizeof(u16));
				}
			}

			// Collision Normals
			{
				u32 numCollisionNormals = static_cast<u32>(collisionNormals.size());
				output.write(reinterpret_cast<const char*>(&numCollisionNormals), sizeof(u32));

				if (numCollisionNormals > 0)
				{
					output.write(reinterpret_cast<const char*>(&collisionNormals[0]), numCollisionNormals * sizeof(std::array<u8, 2>));
				}
			}

			// Collision AABB
			{
				output.write(reinterpret_cast<const char*>(&aabbCenter), sizeof(vec3));
				output.write(reinterpret_cast<const char*>(&aabbExtents), sizeof(vec3));
			}
		}

		// Write CullingData
		{
			output.write(reinterpret_cast<const char*>(&cullingData), sizeof(CullingData));
		}

		// Write Model Data
		{
			// Write Vertex Lookup IDs
			{
				u32 numVertexLookupIds = static_cast<u32>(modelData.vertexLookupIDs.size());
				output.write(reinterpret_cast<const char*>(&numVertexLookupIds), sizeof(u32));

				if (numVertexLookupIds > 0)
				{
					output.write(reinterpret_cast<const char*>(&modelData.vertexLookupIDs[0]), numVertexLookupIds * sizeof(u16));
				}
			}

			// Write Indices
			{
				u32 numIndices = static_cast<u32>(modelData.indices.size());
				output.write(reinterpret_cast<const char*>(&numIndices), sizeof(u32));

				if (numIndices > 0)
				{
					output.write(reinterpret_cast<const char*>(&modelData.indices[0]), numIndices * sizeof(u16));
				}
			}

			// Write Render Batches
			{
				u32 numRenderBatches = static_cast<u32>(modelData.renderBatches.size());
				output.write(reinterpret_cast<const char*>(&numRenderBatches), sizeof(u32));

				for (u32 i = 0; i < numRenderBatches; i++)
				{
					ComplexModel::RenderBatch& renderBatch = modelData.renderBatches[i];

					output.write(reinterpret_cast<const char*>(&renderBatch.groupID), sizeof(u16));
					output.write(reinterpret_cast<const char*>(&renderBatch.vertexStart), sizeof(u32));
					output.write(reinterpret_cast<const char*>(&renderBatch.vertexCount), sizeof(u32));
					output.write(reinterpret_cast<const char*>(&renderBatch.indexStart), sizeof(u32));
					output.write(reinterpret_cast<const char*>(&renderBatch.indexCount), sizeof(u32));

					u32 numTextureUnits = static_cast<u32>(renderBatch.textureUnits.size());
					output.write(reinterpret_cast<const char*>(&numTextureUnits), sizeof(u32));

					for (u32 j = 0; j < numTextureUnits; j++)
					{
						ComplexModel::TextureUnit& textureUnit = renderBatch.textureUnits[j];

						output.write(reinterpret_cast<const char*>(&textureUnit.flags), sizeof(ComplexModel::TextureUnit::Flags));
						output.write(reinterpret_cast<const char*>(&textureUnit.shaderID), sizeof(u16));
						output.write(reinterpret_cast<const char*>(&textureUnit.materialIndex), sizeof(u16));
						output.write(reinterpret_cast<const char*>(&textureUnit.materialLayer), sizeof(u16));
						output.write(reinterpret_cast<const char*>(&textureUnit.textureCount), sizeof(u16));
						output.write(reinterpret_cast<const char*>(&textureUnit.textureIndexStart), sizeof(u16));
						output.write(reinterpret_cast<const char*>(&textureUnit.textureTransformIndexStart), sizeof(u16));
						output.write(reinterpret_cast<const char*>(&textureUnit.textureUnitLookupID), sizeof(u16));
					}
				}
			}
		}

		return true;
	}
	bool ComplexModel::Read(std::shared_ptr<Bytebuffer>& buffer, ComplexModel& out)
	{
		out = { };

		// Read Header
		{
			if (!buffer->Get(out.header))
				return false;

			if (!buffer->Get(out.flags))
				return false;
		}

		// Read Sequences
		{
			if (!ReadVectorFromBuffer(buffer, out.sequences))
				return false;
		}

		// Read Bones
		{
			u32 numElements = 0;
			if (!buffer->GetU32(numElements))
				return false;

			if (numElements)
			{
				out.bones.resize(numElements);
				for (u32 i = 0; i < numElements; i++)
				{
					ComplexModel::Bone& bone = out.bones[i];

					if (!buffer->GetI32(bone.primaryBoneIndex))
						return false;

					if (!buffer->Get(bone.flags))
						return false;

					if (!buffer->GetI16(bone.parentBoneID))
						return false;

					if (!buffer->GetU16(bone.submeshID))
						return false;

					if (!bone.translation.Deserialize(buffer.get()))
						return false;

					if (!bone.rotation.Deserialize(buffer.get()))
						return false;

					if (!bone.scale.Deserialize(buffer.get()))
						return false;

					if (!buffer->Get(bone.pivot))
						return false;
				}
			}
		}

		// Read Vertices
		{
			if (!ReadVectorFromBuffer(buffer, out.vertices))
				return false;
		}

		// Read Textures
		{
			if (!ReadVectorFromBuffer(buffer, out.textures))
				return false;
		}

		// Read Material
		{
			if (!ReadVectorFromBuffer(buffer, out.materials))
				return false;
		}

		// Read Texture Transforms
		{
			u32 numElements = 0;
			if (!buffer->GetU32(numElements))
				return false;

			if (numElements)
			{
				out.textureTransforms.resize(numElements);
				for (u32 i = 0; i < numElements; i++)
				{
					ComplexModel::TextureTransform& textureTransform = out.textureTransforms[i];

					if (!textureTransform.translation.Deserialize(buffer.get()))
						return false;

					if (!textureTransform.rotation.Deserialize(buffer.get()))
						return false;

					if (!textureTransform.scale.Deserialize(buffer.get()))
						return false;
				}
			}
		}

		// Read Texture Index Lookup Table
		{
			if (!ReadVectorFromBuffer(buffer, out.textureIndexLookupTable))
				return false;
		}

		// Read Texture Unit Lookup Table
		{
			if (!ReadVectorFromBuffer(buffer, out.textureUnitLookupTable))
				return false;
		}

		// Read Texture Transparency Lookup Table
		{
			if (!ReadVectorFromBuffer(buffer, out.textureTransparencyLookupTable))
				return false;
		}

		// Read Texture Transform Lookup Table
		{
			if (!ReadVectorFromBuffer(buffer, out.textureTransformLookupTable))
				return false;
		}

		// Read Texture Combiner Combos
		{
			if (!ReadVectorFromBuffer(buffer, out.textureCombinerCombos))
				return false;
		}

		// Read Collision Data
		{
			// Read Vertices
			{
				if (!ReadVectorFromBuffer(buffer, out.collisionVertexPositions))
					return false;
			}

			// Read Indices
			{
				if (!ReadVectorFromBuffer(buffer, out.collisionIndices))
					return false;
			}

			// Read Normals
			{
				if (!ReadVectorFromBuffer(buffer, out.collisionNormals))
					return false;
			}

			// Read AABB
			{
				if (!buffer->Get(out.aabbCenter))
					return false;

				if (!buffer->Get(out.aabbExtents))
					return false;
			}
		}

		// Read Culling Data
		{
			if (!buffer->Get(out.cullingData))
				return false;
		}

		// Read Model Data
		{
			// Read Vertex Lookup IDs
			{
				if (!ReadVectorFromBuffer(buffer, out.modelData.vertexLookupIDs))
					return false;
			}

			// Read Indices
			{
				if (!ReadVectorFromBuffer(buffer, out.modelData.indices))
					return false;
			}

			// Read Render Batches
			{
				u32 numRenderBatches = 0;
				if (!buffer->GetU32(numRenderBatches))
					return false;

				if (numRenderBatches)
				{
					out.modelData.renderBatches.resize(numRenderBatches);
					for (u32 i = 0; i < numRenderBatches; i++)
					{
						ComplexModel::RenderBatch& renderBatch = out.modelData.renderBatches[i];

						if (!buffer->GetU16(renderBatch.groupID))
							return false;

						if (!buffer->GetU32(renderBatch.vertexStart))
							return false;

						if (!buffer->GetU32(renderBatch.vertexCount))
							return false;

						if (!buffer->GetU32(renderBatch.indexStart))
							return false;

						if (!buffer->GetU32(renderBatch.indexCount))
							return false;

						u32 numTextureUnits = 0;
						if (!buffer->GetU32(numTextureUnits))
							return false;

						if (numTextureUnits)
						{
							renderBatch.textureUnits.resize(numTextureUnits);
							for (u32 j = 0; j < numTextureUnits; j++)
							{
								ComplexModel::TextureUnit& textureUnit = renderBatch.textureUnits[j];

								if (!buffer->Get(textureUnit.flags))
									return false;

								if (!buffer->GetI16(textureUnit.shaderID))
									return false;

								if (!buffer->GetU16(textureUnit.materialIndex))
									return false;

								if (!buffer->GetU16(textureUnit.materialLayer))
									return false;

								if (!buffer->GetU16(textureUnit.textureCount))
									return false;

								if (!buffer->GetU16(textureUnit.textureIndexStart))
									return false;

								if (!buffer->GetU16(textureUnit.textureTransformIndexStart))
									return false;

								if (!buffer->GetU16(textureUnit.textureUnitLookupID))
									return false;
							}
						}
					}
				}
			}
		}

		return true;
	}

	bool ComplexModel::FromM2(std::shared_ptr<Bytebuffer>& rootBuffer, std::shared_ptr<Bytebuffer>& skinBuffer, Layout& layout, ComplexModel& out)
	{
		out.flags = *reinterpret_cast<ComplexModel::Flags*>(&layout.md21.flags);

		// TODO : Global Sequences, Sequences

		// Read Bones
		{
			u32 numBones = layout.md21.bones.size;

			out.bones.resize(numBones);
			for (u32 i = 0; i < numBones; i++)
			{
				ComplexModel::Bone& bone = out.bones[i];
				M2CompBone* m2Bone = layout.md21.bones.GetElement(rootBuffer, i);

				bone.primaryBoneIndex = m2Bone->keyBoneID;
				bone.flags = *reinterpret_cast<ComplexModel::Bone::Flags*>(&m2Bone->flags);
				bone.parentBoneID = m2Bone->parentBone;
				bone.submeshID = m2Bone->submeshID;

				// TODO : Read Translation Track
				{

				}

				// TODO : Read Rotation Track
				{

				}

				// TODO : Read Scale Track
				{

				}

				bone.pivot = m2Bone->pivot;
			}
		}

		// Read Vertices
		{
			u32 numVertices = layout.md21.vertices.size;
			out.vertices.resize(numVertices);

			vec3 aabbMin = vec3(10000.0f, 10000.0f, 10000.0f);
			vec3 aabbMax = vec3(-10000.0f, -10000.0f, -10000.0f);

			for (u32 i = 0; i < numVertices; i++)
			{
				ComplexModel::Vertex& vertex = out.vertices[i];
				M2Vertex* m2Vertex = layout.md21.vertices.GetElement(rootBuffer, i);

				vertex.position = m2Vertex->position;
				vertex.uvCoords[0] = m2Vertex->uvCords[0];
				vertex.uvCoords[1] = m2Vertex->uvCords[1];

				vec3 normal = glm::normalize(m2Vertex->normal);
				vec2 octNormal = OctNormalEncode(normal);
				vertex.octNormal[0] = static_cast<u8>(octNormal.x * 255.0f);
				vertex.octNormal[1] = static_cast<u8>(octNormal.y * 255.0f);

				for (u32 j = 0; j < 4; j++)
				{
					vertex.boneIndices[j] = m2Vertex->boneIndices[j];
					vertex.boneWeights[j] = m2Vertex->boneWeights[j];
				}

				for (u32 j = 0; j < 3; j++)
				{
					if (vertex.position[j] < aabbMin[j])
						aabbMin[j] = vertex.position[j];

					if (vertex.position[j] > aabbMax[j])
						aabbMax[j] = vertex.position[j];
				}
			}

			out.cullingData.center = (aabbMin + aabbMax) * 0.5f;
			out.cullingData.extents = hvec3(aabbMax) - out.cullingData.center;
			out.cullingData.boundingSphereRadius = layout.md21.cullingAABBBounds.radius;
		}

		// Read Textures
		{
			u32 numTextures = layout.md21.textures.size;

			out.textures.resize(numTextures);
			for (u32 i = 0; i < numTextures; i++)
			{
				ComplexModel::Texture& texture = out.textures[i];
				M2Texture* m2Texture = layout.md21.textures.GetElement(rootBuffer, i);

				texture.type = static_cast<ComplexModel::Texture::Type>(m2Texture->type);
				texture.flags = *reinterpret_cast<ComplexModel::Texture::Flags*>(&m2Texture->flags);

				if (i < layout.txid.textureFileIDs.size())
				{
					texture.textureHash = layout.txid.textureFileIDs[i];
				}
			}
		}

		// Read Materials
		{
			u32 numMaterials = layout.md21.materials.size;
			u32 materialsOffset = layout.md21.materials.offset;

			out.materials.resize(numMaterials);
			memcpy(out.materials.data(), &rootBuffer->GetDataPointer()[materialsOffset], numMaterials * sizeof(ComplexModel::Material));
		}

		// TODO : Texture Transforms

		// Read Texture Lookup Tables
		{
			// Read Texture Index Lookup Table
			{
				u32 numTextureLookupIDs = layout.md21.textureCombinationList.size;
				u32 textureLookupIDsOffset = layout.md21.textureCombinationList.offset;

				out.textureIndexLookupTable.resize(numTextureLookupIDs);
				memcpy(out.textureIndexLookupTable.data(), &rootBuffer->GetDataPointer()[textureLookupIDsOffset], numTextureLookupIDs * sizeof(u16));
			}

			// Read Texture Unit Lookup Table
			{
				u32 numTextureUnitLookupIDs = layout.md21.textureUnitLookupList.size;
				u32 textureUnitLookupIDsOffset = layout.md21.textureUnitLookupList.offset;

				out.textureUnitLookupTable.resize(numTextureUnitLookupIDs);
				memcpy(out.textureUnitLookupTable.data(), &rootBuffer->GetDataPointer()[textureUnitLookupIDsOffset], numTextureUnitLookupIDs * sizeof(u16));
			}

			// Read Texture Transparency Lookup Table
			{
				u32 numTextureTransparencyLookupIDs = layout.md21.textureTransparencyLookupList.size;
				u32 textureTransparencyLookupIDsOffset = layout.md21.textureTransparencyLookupList.offset;

				out.textureTransparencyLookupTable.resize(numTextureTransparencyLookupIDs);
				memcpy(out.textureTransparencyLookupTable.data(), &rootBuffer->GetDataPointer()[textureTransparencyLookupIDsOffset], numTextureTransparencyLookupIDs * sizeof(u16));
			}

			// Read Texture Transform Lookup Table
			{
				u32 numTextureUVAnimationLookupIDs = layout.md21.textureUVAnimationLookupList.size;
				u32 textureUVAnimationLookupIDsOffset = layout.md21.textureUVAnimationLookupList.offset;

				out.textureTransformLookupTable.resize(numTextureUVAnimationLookupIDs);
				memcpy(out.textureTransformLookupTable.data(), &rootBuffer->GetDataPointer()[textureUVAnimationLookupIDsOffset], numTextureUVAnimationLookupIDs * sizeof(u16));
			}

			// Read Texture Combiner Combos
			{
				if (layout.md21.flags.UseTextureCombinerCombos)
				{
					u32 numTextureCombinerCombos = layout.md21.textureCombinerCombos.size;
					u32 textureCombinerCombosOffset = layout.md21.textureCombinerCombos.offset;

					out.textureCombinerCombos.resize(numTextureCombinerCombos);
					memcpy(out.textureCombinerCombos.data(), &rootBuffer->GetDataPointer()[textureCombinerCombosOffset], numTextureCombinerCombos * sizeof(u16));
				}
			}
		}

		// Read Model Data
		{
			// Read Vertex Lookup Ids
			{
				u32 numVertexLookupIDs = layout.skin.vertices.size;
				u32 vertexLookupIDsOffset = layout.skin.vertices.offset;

				out.modelData.vertexLookupIDs.resize(numVertexLookupIDs);
				memcpy(out.modelData.vertexLookupIDs.data(), &skinBuffer->GetDataPointer()[vertexLookupIDsOffset], numVertexLookupIDs * sizeof(u16));
			}

			// Read Indices
			{
				u32 numIndicies = layout.skin.indices.size;
				u32 indicesOffset = layout.skin.indices.offset;

				out.modelData.indices.resize(numIndicies);
				memcpy(out.modelData.indices.data(), &skinBuffer->GetDataPointer()[indicesOffset], numIndicies * sizeof(u16));
			}

			// Read Render Batches
			{
				u32 numRenderBatches = layout.skin.subMeshes.size;
				u32 numTextureUnits = layout.skin.batches.size;

				for (u32 i = 0; i < numRenderBatches; i++)
				{
					ComplexModel::RenderBatch& renderBatch = out.modelData.renderBatches.emplace_back();
					M2SkinSelection* m2SkinSelection = layout.skin.subMeshes.GetElement(skinBuffer, i);

					renderBatch.groupID = m2SkinSelection->skinSectionID;
					renderBatch.vertexStart = m2SkinSelection->vertexStart;
					renderBatch.vertexCount = m2SkinSelection->vertexCount;
					renderBatch.indexStart = m2SkinSelection->indexStart + (m2SkinSelection->level * std::numeric_limits<u16>().max()); // "Level" is an incremental value that goes up once when indexStart goes above the numeric limit of u16 (65536)
					renderBatch.indexCount = m2SkinSelection->indexCount;

					// Read Texture Units
					{
						renderBatch.textureUnits.reserve(8); // Wowdev.wiki states it is capped at 7 (0 based)

						for (u32 j = 0; j < numTextureUnits; j++)
						{
							M2Batch* m2Batch = layout.skin.batches.GetElement(skinBuffer, j);

							if (m2Batch->skinSectionIndex != i)
								continue;

							ComplexModel::TextureUnit& textureUnit = renderBatch.textureUnits.emplace_back();
							textureUnit.flags = *reinterpret_cast<ComplexModel::TextureUnit::Flags*>(&m2Batch->flags);
							textureUnit.shaderID = m2Batch->shaderID;
							textureUnit.materialIndex = m2Batch->materialIndex;
							textureUnit.materialLayer = m2Batch->materialLayer;

							textureUnit.textureCount = m2Batch->textureCount;
							textureUnit.textureIndexStart = m2Batch->textureLookupID;
							textureUnit.textureTransformIndexStart = m2Batch->textureUVAnimationLookupID;

							/*for (u16 t = 0; t < textureUnit.textureCount; t++)
							{
								textureUnit.textureIndices[t] = out.textureIndexLookupTable[t + m2Batch->textureLookupID];
								textureUnit.textureTransformIndices[t] = out.textureTransformLookupTable[t + m2Batch->textureUVAnimationLookupID];
							}*/

							textureUnit.textureTransparencyLookupID = m2Batch->textureTransparencyLookupID;

							// Check priorityPlane and take the biggest one for the batch
							u8 renderPriority = static_cast<u8>(static_cast<i16>(m2Batch->priorityPlane) + 127);
							if (renderPriority > renderBatch.renderPriority)
							{
								renderBatch.renderPriority = renderPriority;
							}
						}
					}
				}
			}

			// Read Collision Data
			{
				u32 numCollisionVertices = layout.md21.collisionVertices.size;
				out.collisionVertexPositions.resize(numCollisionVertices);

				vec3 aabbMin = vec3(10000.0f, 10000.0f, 10000.0f);
				vec3 aabbMax = vec3(-10000.0f, -10000.0f, -10000.0f);

				for (u32 i = 0; i < numCollisionVertices; i++)
				{
					vec3 m2CollisionVertexPosition = *layout.md21.collisionVertices.GetElement(rootBuffer, i);

					vec3& vertex = out.collisionVertexPositions[i];
					vertex = m2CollisionVertexPosition; // vec3(-m2CollisionVertexPosition.x, -m2CollisionVertexPosition.y, m2CollisionVertexPosition.z);

					for (u32 j = 0; j < 3; j++)
					{
						if (vertex[j] < aabbMin[j])
							aabbMin[j] = vertex[j];

						if (vertex[j] > aabbMax[j])
							aabbMax[j] = vertex[j];
					}
				}

				out.aabbCenter = (aabbMin + aabbMax) * 0.5f;
				out.aabbExtents = aabbMax - out.aabbCenter;

				u32 numCollisionIndices = layout.md21.collisionIndices.size;
				out.collisionIndices.resize(numCollisionIndices);
				for (u32 i = 0; i < numCollisionIndices; i++)
				{
					u16* m2CollisionIndex = layout.md21.collisionIndices.GetElement(rootBuffer, i);

					out.collisionIndices[i] = *m2CollisionIndex;
				}

				u32 numCollisionNormals = layout.md21.collisionNormals.size;
				out.collisionNormals.resize(numCollisionNormals);
				for (u32 i = 0; i < numCollisionNormals; i++)
				{
					vec3* m2CollisionNormal = layout.md21.collisionNormals.GetElement(rootBuffer, i);

					vec3 normal = glm::normalize(*m2CollisionNormal);
					vec2 octNormal = OctNormalEncode(normal);

					std::array<u8, 2>& normalArr = out.collisionNormals[i];
					normalArr[0] = static_cast<u8>(octNormal.x * 255.0f);
					normalArr[1] = static_cast<u8>(octNormal.y * 255.0f);
				}
			}
		}

		// Post Processing On CModel Data
		{
			// Adjust Indicies to refer to the global vertex list
			for (u32 i = 0; i < out.modelData.indices.size(); i++)
			{
				u16& localVertexIndex = out.modelData.indices[i];

				if (localVertexIndex >= out.modelData.vertexLookupIDs.size())
					return false;

				localVertexIndex = out.modelData.vertexLookupIDs[localVertexIndex];
			}

			// TODO : Clone and invert two sided renderbatches so we don't need to have double pipelines

			// Fix Shader ID
			{
				u32 numRenderBatches = static_cast<u32>(out.modelData.renderBatches.size());
				for (u32 i = 0; i < numRenderBatches; i++)
				{
					ComplexModel::RenderBatch& renderBatch = out.modelData.renderBatches[i];

					u32 numTextureUnits = static_cast<u32>(renderBatch.textureUnits.size());
					for (u32 j = 0; j < numTextureUnits; j++)
					{
						ComplexModel::TextureUnit& textureUnit = renderBatch.textureUnits[j];

						i16 shaderID = textureUnit.shaderID;
						textureUnit.vertexShaderID = GetVertexShaderID(shaderID, textureUnit.textureCount);
						textureUnit.pixelShaderID = GetPixelShaderID(shaderID, textureUnit.textureCount);
					}
				}
			}
		}

		return true;
	}

	enum class M2PixelShader : int {
		//Wotlk deprecated shaders
		Combiners_Decal = -1,
		Combiners_Add = -2,
		Combiners_Mod2x = -3,
		Combiners_Fade = -4,
		Combiners_Opaque_Add = -5,
		Combiners_Opaque_AddNA = -6,
		Combiners_Add_Mod = -7,
		Combiners_Mod2x_Mod2x = -8,

		//Legion modern shaders
		Combiners_Opaque = 0,
		Combiners_Mod = 1,
		Combiners_Opaque_Mod = 2,
		Combiners_Opaque_Mod2x = 3,
		Combiners_Opaque_Mod2xNA = 4,
		Combiners_Opaque_Opaque = 5,
		Combiners_Mod_Mod = 6,
		Combiners_Mod_Mod2x = 7,
		Combiners_Mod_Add = 8,
		Combiners_Mod_Mod2xNA = 9,
		Combiners_Mod_AddNA = 10,
		Combiners_Mod_Opaque = 11,
		Combiners_Opaque_Mod2xNA_Alpha = 12,
		Combiners_Opaque_AddAlpha = 13,
		Combiners_Opaque_AddAlpha_Alpha = 14,
		Combiners_Opaque_Mod2xNA_Alpha_Add = 15,
		Combiners_Mod_AddAlpha = 16,
		Combiners_Mod_AddAlpha_Alpha = 17,
		Combiners_Opaque_Alpha_Alpha = 18,
		Combiners_Opaque_Mod2xNA_Alpha_3s = 19,
		Combiners_Opaque_AddAlpha_Wgt = 20,
		Combiners_Mod_Add_Alpha = 21,
		Combiners_Opaque_ModNA_Alpha = 22,
		Combiners_Mod_AddAlpha_Wgt = 23,
		Combiners_Opaque_Mod_Add_Wgt = 24,
		Combiners_Opaque_Mod2xNA_Alpha_UnshAlpha = 25,
		Combiners_Mod_Dual_Crossfade = 26,
		Combiners_Opaque_Mod2xNA_Alpha_Alpha = 27,
		Combiners_Mod_Masked_Dual_Crossfade = 28,
		Combiners_Opaque_Alpha = 29,
		Guild = 30,
		Guild_NoBorder = 31,
		Guild_Opaque = 32,
		Combiners_Mod_Depth = 33,
		Illum = 34,
		Combiners_Mod_Mod_Mod_Const = 35,
		NewUnkCombiner = 36
	};

	enum class M2VertexShader : int {
		Diffuse_T1 = 0,
		Diffuse_Env = 1,
		Diffuse_T1_T2 = 2,
		Diffuse_T1_Env = 3,
		Diffuse_Env_T1 = 4,
		Diffuse_Env_Env = 5,
		Diffuse_T1_Env_T1 = 6,
		Diffuse_T1_T1 = 7,
		Diffuse_T1_T1_T1 = 8,
		Diffuse_EdgeFade_T1 = 9,
		Diffuse_T2 = 10,
		Diffuse_T1_Env_T2 = 11,
		Diffuse_EdgeFade_T1_T2 = 12,
		Diffuse_EdgeFade_Env = 13,
		Diffuse_T1_T2_T1 = 14,
		Diffuse_T1_T2_T3 = 15,
		Color_T1_T2_T3 = 16,
		BW_Diffuse_T1 = 17,
		BW_Diffuse_T1_T2 = 18,
	};

	inline constexpr const int operator+ (M2PixelShader const val) { return static_cast<const int>(val); };
	inline constexpr const int operator+ (M2VertexShader const val) { return static_cast<const int>(val); };

	struct M2Shaders {
		unsigned int pixel;
		unsigned int vertex;
		unsigned int hull;
		unsigned int domain;
	};
	static std::array<M2Shaders, 36> M2ShaderTable = { {
			{ +M2PixelShader::Combiners_Opaque_Mod2xNA_Alpha,              +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_AddAlpha,                   +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_AddAlpha_Alpha,             +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_Mod2xNA_Alpha_Add,          +M2VertexShader::Diffuse_T1_Env_T1, 2, 2 },
			{ +M2PixelShader::Combiners_Mod_AddAlpha,                      +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_AddAlpha,                   +M2VertexShader::Diffuse_T1_T1, 1, 1 },
			{ +M2PixelShader::Combiners_Mod_AddAlpha,                      +M2VertexShader::Diffuse_T1_T1, 1, 1 },
			{ +M2PixelShader::Combiners_Mod_AddAlpha_Alpha,                +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_Alpha_Alpha,                +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_Mod2xNA_Alpha_3s,           +M2VertexShader::Diffuse_T1_Env_T1, 2, 2 },
			{ +M2PixelShader::Combiners_Opaque_AddAlpha_Wgt,               +M2VertexShader::Diffuse_T1_T1, 1, 1 },
			{ +M2PixelShader::Combiners_Mod_Add_Alpha,                     +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_ModNA_Alpha,                +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Mod_AddAlpha_Wgt,                  +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Mod_AddAlpha_Wgt,                  +M2VertexShader::Diffuse_T1_T1, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_AddAlpha_Wgt,               +M2VertexShader::Diffuse_T1_T2, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_Mod_Add_Wgt,                +M2VertexShader::Diffuse_T1_Env, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_Mod2xNA_Alpha_UnshAlpha,    +M2VertexShader::Diffuse_T1_Env_T1, 2, 2 },
			{ +M2PixelShader::Combiners_Mod_Dual_Crossfade,                +M2VertexShader::Diffuse_T1, 0, 0 },
			{ +M2PixelShader::Combiners_Mod_Depth,                         +M2VertexShader::Diffuse_EdgeFade_T1, 0, 0 },
			{ +M2PixelShader::Combiners_Opaque_Mod2xNA_Alpha_Alpha,        +M2VertexShader::Diffuse_T1_Env_T2, 2, 2 },
			{ +M2PixelShader::Combiners_Mod_Mod,                           +M2VertexShader::Diffuse_EdgeFade_T1_T2, 1, 1 },
			{ +M2PixelShader::Combiners_Mod_Masked_Dual_Crossfade,         +M2VertexShader::Diffuse_T1_T2, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_Alpha,                      +M2VertexShader::Diffuse_T1_T1, 1, 1 },
			{ +M2PixelShader::Combiners_Opaque_Mod2xNA_Alpha_UnshAlpha,    +M2VertexShader::Diffuse_T1_Env_T2, 2, 2 },
			{ +M2PixelShader::Combiners_Mod_Depth,                         +M2VertexShader::Diffuse_EdgeFade_Env, 0, 0 },
			{ +M2PixelShader::Guild,                                       +M2VertexShader::Diffuse_T1_T2_T1, 2, 1 },
			{ +M2PixelShader::Guild_NoBorder,                              +M2VertexShader::Diffuse_T1_T2, 1, 2 },
			{ +M2PixelShader::Guild_Opaque,                                +M2VertexShader::Diffuse_T1_T2_T1, 2, 1 },
			{ +M2PixelShader::Illum,                                       +M2VertexShader::Diffuse_T1_T1, 1, 1 },
			{ +M2PixelShader::Combiners_Mod_Mod_Mod_Const,                 +M2VertexShader::Diffuse_T1_T2_T3, 2, 2 },
			{ +M2PixelShader::Combiners_Mod_Mod_Mod_Const,                 +M2VertexShader::Color_T1_T2_T3, 2, 2 },
			{ +M2PixelShader::Combiners_Opaque,                            +M2VertexShader::Diffuse_T1, 0, 0 },
			{ +M2PixelShader::Combiners_Mod_Mod2x,                         +M2VertexShader::Diffuse_EdgeFade_T1_T2, 1, 1 },
			{ +M2PixelShader::Combiners_Mod,                               +M2VertexShader::Diffuse_EdgeFade_T1, 1, 1 },
			{ +M2PixelShader::NewUnkCombiner,                              +M2VertexShader::Diffuse_EdgeFade_T1_T2, 1, 1 },
	} };

	i8 ComplexModel::GetVertexShaderID(i16 shaderID, u16 textureCount)
	{
		i8 result = -1;

		if (shaderID < 0)
		{
			i32 shaderTableEntry = shaderID & 0x7FFF;

			if (shaderTableEntry >= static_cast<i32>(M2ShaderTable.size()))
				return false;

			result = M2ShaderTable[shaderTableEntry].vertex;
		}
		else if (textureCount == 1)
		{
			if ((shaderID & 0x80) != 0)
			{
				result = 1;
			}
			else
			{
				result = 10;

				if (!(shaderID & 0x4000))
					result = 0;
			}
		}
		else if ((shaderID & 0x80) != 0)
		{
			result = ((shaderID & 0x8) >> 3) | 4;
		}
		else
		{
			result = 3;

			if (!(shaderID & 0x8))
				result = 5 * ((shaderID & 0x4000) == 0) + 2;
		}

		return result;
	}
	i8 ComplexModel::GetPixelShaderID(i16 shaderID, u16 textureCount)
	{
		i8 result = -1;

		if (shaderID < 0)
		{
			i32 shaderTableEntry = shaderID & 0x7FFF;

			if (shaderTableEntry >= static_cast<i32>(M2ShaderTable.size()))
				return false;

			result = M2ShaderTable[shaderTableEntry].pixel;
		}
		else if (textureCount == 1)
		{
			result = (shaderID & 0x70) != 0;
		}
		else
		{
			if (shaderID & 0x70)
			{
				switch (shaderID & 0x7)
				{
					case 3:
					{
						result = +M2PixelShader::Combiners_Mod_Add;
						break;
					}
					case 4:
					{
						result = +M2PixelShader::Combiners_Mod_Mod2x;
						break;
					}
					case 6:
					{
						result = +M2PixelShader::Combiners_Mod_Mod2xNA;
						break;
					}
					case 7:
					{
						result = +M2PixelShader::Combiners_Mod_AddNA;
						break;
					}

					default:
					{
						result = +M2PixelShader::Combiners_Mod_Mod;
						break;
					}
				}
			}
			else
			{

				switch (shaderID & 0x7)
				{
					case 0:
					{
						result = +M2PixelShader::Combiners_Opaque_Opaque;
						break;
					}
					case 3:
					case 7:
					{
						result = +M2PixelShader::Combiners_Opaque_AddAlpha;
						break;
					}
					case 4:
					{
						result = +M2PixelShader::Combiners_Opaque_Mod2x;
						break;
					}
					case 6:
					{
						result = +M2PixelShader::Combiners_Opaque_Mod2xNA;
						break;
					}

					default:
					{
						result = +M2PixelShader::Combiners_Opaque_Mod;
						break;
					}
				}
			}
		}

		return result;
	}
}