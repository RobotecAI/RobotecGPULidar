#pragma once

#include <cinttypes>
#include <nvToolsExt.h>
#include <optional>

constexpr uint32_t NVTX_CAT_API = 0; // Positive numbers = graphOrdinal
constexpr uint32_t NVTX_COL_SYNC = 0xffffd700;
constexpr uint32_t NVTX_COL_SYNC_CPU = 0xffdaa520;
constexpr uint32_t NVTX_COL_WORK = 0xff90ee90;
constexpr uint32_t NVTX_COL_CALL = 0xff9370db;


struct NvtxRange
{
	template<typename... Args>
	NvtxRange(uint32_t category, uint32_t color, fmt::format_string<Args...> fmt, Args... args)
	{
		auto msg = fmt::format(fmt, std::forward<Args>(args)...);
		nvtxEventAttributes_t eventAttributes = {0};
		eventAttributes.version = NVTX_VERSION;
		eventAttributes.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
		eventAttributes.colorType = NVTX_COLOR_ARGB;
		eventAttributes.color = color;
		eventAttributes.category = category;
		eventAttributes.messageType = NVTX_MESSAGE_TYPE_ASCII;
		eventAttributes.message.ascii = msg.c_str();
		correlationId = nvtxRangeStartEx(&eventAttributes);
	}

	NvtxRange(const NvtxRange&) = delete;
	NvtxRange(NvtxRange&&) = delete;
	NvtxRange& operator=(const NvtxRange&) = delete;
	NvtxRange& operator=(NvtxRange&&) = delete;

	~NvtxRange()
	{
		nvtxRangeEnd(correlationId);
	}
private:
	nvtxRangeId_t correlationId;
};
