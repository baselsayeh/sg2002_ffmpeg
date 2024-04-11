/*
 * OMX Video encoder
 * Copyright (C) 2011 Martin Storsjo
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config.h"

/*#if CONFIG_OMX_RPI
#define OMX_SKIP64BIT
#endif*/

#include <dlfcn.h>
//#include <OMX_Core.h>
//#include <OMX_Component.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "libavutil/avstring.h"
#include "libavutil/avutil.h"
#include "libavutil/common.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"

#include <cvi_sys.h>
#include <cvi_vb.h>
#include <cvi_venc.h>

#include "avcodec.h"
#include "codec_internal.h"
#include "encode.h"
#include "h264.h"
#include "pthread_internal.h"

#define ALIGN(x, a)      (((x) + ((a)-1)) & ~((a)-1))

typedef struct SophgoCodecContext {
    const AVClass *class;
    //OMXContext *omx_context;

    AVCodecContext *avctx;

    VIDEO_FRAME_INFO_S frame;

    VB_POOL enc_pool;
    VB_BLK enc_block;

    bool receive_ready;

    /*CVI_U32 frame_y_stride;
    CVI_U32 frame_uv_stride;
    CVI_U32 frame_y_size;
    CVI_U32 frame_uv_size;
    CVI_U32 frame_total_size;*/

    int input_zerocopy;
    int profile;
} SophgoCodecContext;

static av_cold int sophgo_venc_init(SophgoCodecContext *s) {
    int ret;
    VENC_CHN_ATTR_S ChannelAttr; //Encoding parameters
    VENC_RC_PARAM_S rc_params; //QP params
    AVCodecContext *avctx = s->avctx;
    unsigned int max_bitrate;

    max_bitrate = (avctx->bit_rate&0xFFFFFFFF)/1000; //in kbps
    if (avctx->bit_rate < 100000 | max_bitrate < 100) {
        av_log(avctx, AV_LOG_ERROR, "too low bitrate: %u\n", max_bitrate);
        return -1;
    }
    av_log(avctx, AV_LOG_INFO, "using bitrate: %u (kbps)\n", max_bitrate);

    ChannelAttr.stVencAttr.enType = PT_H264;
    ChannelAttr.stVencAttr.u32BufSize = 0; //IN EXAMPLE, IT IS ZERO //TODO
    ChannelAttr.stVencAttr.u32MaxPicWidth = s->avctx->width;
    ChannelAttr.stVencAttr.u32MaxPicHeight = s->avctx->height;
    ChannelAttr.stVencAttr.u32PicWidth = s->avctx->width;
    ChannelAttr.stVencAttr.u32PicHeight = s->avctx->height;
    ChannelAttr.stVencAttr.bByFrame = CVI_TRUE; //TODO

    ChannelAttr.stVencAttr.bSingleCore = CVI_FALSE;
    ChannelAttr.stVencAttr.bEsBufQueueEn = CVI_TRUE; //TODO
    ChannelAttr.stVencAttr.bIsoSendFrmEn = CVI_TRUE; //TODO

    switch (s->profile) {
        case FF_PROFILE_H264_BASELINE:
            ChannelAttr.stVencAttr.u32Profile = H264E_PROFILE_BASELINE;
            break;
        case FF_PROFILE_H264_MAIN:
            ChannelAttr.stVencAttr.u32Profile = H264E_PROFILE_MAIN;
            break;
        case FF_PROFILE_H264_HIGH:
            ChannelAttr.stVencAttr.u32Profile = H264E_PROFILE_HIGH;
            break;
        default:
            av_log(avctx, AV_LOG_ERROR, "unsupported profile: %u\n", s->profile);
            return -1;
    }

    av_log(avctx, AV_LOG_WARNING, "framerate: %u\n", avctx->framerate.num);
    av_log(avctx, AV_LOG_WARNING, "bit_rate: %u\n", avctx->bit_rate&0xFFFFFFFF);

    //TODO: ACTUALLY GET PARAMS FROM AVCodecContext
    //TODO: check framerate, bitrate and Q values

    /*
    ChannelAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
    ChannelAttr.stRcAttr.stH264Vbr.u32Gop = 120; //Each 120 frame, insert GOP
    ChannelAttr.stRcAttr.stH264Vbr.u32StatTime = 0;
    /*ChannelAttr.stRcAttr.stH264Vbr.u32SrcFrameRate = 30;
    ChannelAttr.stRcAttr.stH264Vbr.fr32DstFrameRate = 32; //?? IDK why it is two*
    ChannelAttr.stRcAttr.stH264Vbr.u32SrcFrameRate = avctx->framerate.num;
    ChannelAttr.stRcAttr.stH264Vbr.fr32DstFrameRate = avctx->framerate.num + 2; //?? IDK why it is two
    ChannelAttr.stRcAttr.stH264Vbr.u32MaxBitRate = max_bitrate; //???? also idk
    ChannelAttr.stRcAttr.stH264Vbr.bVariFpsEn = CVI_FALSE;
    */
    ChannelAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    ChannelAttr.stRcAttr.stH264Cbr.u32Gop = 120; //Each 120 frame, insert GOP
    ChannelAttr.stRcAttr.stH264Cbr.u32StatTime = 1; //enforce bitrate each 1 second
    ChannelAttr.stRcAttr.stH264Cbr.u32SrcFrameRate = avctx->framerate.num;
    ChannelAttr.stRcAttr.stH264Cbr.fr32DstFrameRate = avctx->framerate.num; //?? IDK why it is two
    ChannelAttr.stRcAttr.stH264Cbr.u32BitRate = max_bitrate; //???? also idk
    ChannelAttr.stRcAttr.stH264Cbr.bVariFpsEn = CVI_FALSE;

    ChannelAttr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    ChannelAttr.stGopAttr.stNormalP.s32IPQpDelta = 2; //TWO in sdk

    ChannelAttr.stVencAttr.stAttrH264e.bRcnRefShareBuf = CVI_TRUE;
    ChannelAttr.stVencAttr.stAttrH264e.bSingleLumaBuf = CVI_TRUE;

    //TODO: maybe create a non-zero channel
    ret = CVI_VENC_CreateChn(0, &ChannelAttr);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Error creating channel: %08x\n", (uint32_t) ret);
        return -1;
    }

    //Set channel attrs
    ret = CVI_VENC_SetChnAttr(0, &ChannelAttr);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Error setting channel attrs: %08x\n", (uint32_t) ret);
        return -1;
    }

    ret = CVI_VENC_GetRcParam(0, &rc_params);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Error getting rc params: %08x\n", (uint32_t) ret);
         return -1;
    }

    /*rc_params.s32FirstFrameStartQp = 18;
    rc_params.stParamH264Vbr.u32MinQp = 18;
    rc_params.stParamH264Vbr.u32MaxQp = 28;
    rc_params.stParamH264Vbr.u32MinIQp = 18;
    rc_params.stParamH264Vbr.u32MaxIQp = 28;

    ret = CVI_VENC_SetRcParam(0, &rc_params);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Error setting rc params: %08x\n", (uint32_t) ret);
        return -1;
    }*/

    return 0;
}

static av_cold int sophgo_vpool_init(SophgoCodecContext *s, unsigned int frame_size) {
    VB_POOL_CONFIG_S pool_config;

    AVCodecContext *avctx = s->avctx;

    pool_config.u32BlkSize = frame_size;
    pool_config.u32BlkCnt = 1;
    pool_config.enRemapMode = VB_REMAP_MODE_NONE;
    snprintf(pool_config.acName, sizeof(pool_config.acName), "ffmpeg_enc1");

    s->enc_pool = CVI_VB_CreatePool(&pool_config);
    if (s->enc_pool == VB_INVALID_HANDLE) {
        av_log(avctx, AV_LOG_ERROR, "Error creating a video pool\n");
        return -1;
    }

    s->enc_block = CVI_VB_GetBlock(s->enc_pool, frame_size);
    if (s->enc_block == VB_INVALID_HANDLE) {
        av_log(avctx, AV_LOG_ERROR, "Error getting a block from the video pool\n");
        return -1;
    }

    return 0;
}

//static av_cold int sophgo_setup_frame(SophgoCodecContext *s, const AVFrame *frame) {
static av_cold int sophgo_setup_frame(SophgoCodecContext *s) {
    int ret;
    unsigned int y_size;
    unsigned int uv_size;
    unsigned int total_size;
    unsigned int frame_size;
    VENC_RECV_PIC_PARAM_S rec_param;
    AVCodecContext *avctx = s->avctx;
    //uint32_t u8BitWidth = 8;

    frame_size = av_image_get_buffer_size(avctx->pix_fmt, avctx->width, avctx->height, 32);

    //Get the frame size
    //s->frame_uv_stride = ALIGN(((s->avctx->width >> 1) * u8BitWidth + 7) >> 3, 32);
    //s->frame_y_stride = s->frame_uv_stride * 2;
    /*s->frame_y_stride = frame->linesize[0];
    s->frame_uv_stride = frame->linesize[1];
    s->frame_y_size = s->frame_y_stride * ALIGN(s->avctx->height, 2);
    s->frame_uv_size = (s->frame_uv_stride*ALIGN(s->avctx->height, 2))>>1; // /2*/

    /*
    y_size = frame->linesize[0] * ALIGN(s->avctx->height, 2);
    uv_size = (frame->linesize[1] * ALIGN(s->avctx->height, 2))>>1;
    */

    y_size = av_image_get_linesize(avctx->pix_fmt, avctx->width, 0) * ALIGN(s->avctx->height, 2);
    uv_size = (av_image_get_linesize(avctx->pix_fmt, avctx->width, 1) * ALIGN(s->avctx->height, 2))>>1;

    /*av_log(avctx, AV_LOG_WARNING, "Warning: line sizes: %u %u %u!!!!\n",
           av_image_get_linesize(avctx->pix_fmt, avctx->width, 0),
           av_image_get_linesize(avctx->pix_fmt, avctx->width, 1),
           av_image_get_linesize(avctx->pix_fmt, avctx->width, 2)
           );*/

    total_size = (uv_size<<1) + y_size;

    if (frame_size != total_size) {
        av_log(avctx, AV_LOG_WARNING, "Warning: frame sizes differ, %u and calculated (by ffmpeg) %u!!!!\n", total_size, frame_size);
    }

    //Create a pool and get a block
    ret = sophgo_vpool_init(s, total_size);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Error initializing video pool\n");
        return ret;
    }

    memset((void *)&s->frame, 0x00, sizeof(VIDEO_FRAME_INFO_S));
    s->frame.stVFrame.u32Width = s->avctx->width;
    s->frame.stVFrame.u32Height = s->avctx->height;

    //s->frame.stVFrame.enPixelFormat = PIXEL_FORMAT_YUV_PLANAR_422; //TODO: from SDK sample prog
    s->frame.stVFrame.enPixelFormat = PIXEL_FORMAT_YUV_PLANAR_420; //TODO: from SDK sample prog
    s->frame.stVFrame.enBayerFormat = BAYER_FORMAT_BG; //TODO: from SDK
    s->frame.stVFrame.enVideoFormat = VIDEO_FORMAT_LINEAR; //TODO: from SDK
    s->frame.stVFrame.enCompressMode = COMPRESS_MODE_NONE;
    s->frame.stVFrame.enDynamicRange = DYNAMIC_RANGE_SDR8; //TODO: Does it support HDR???
    s->frame.stVFrame.enColorGamut = COLOR_GAMUT_BT709; //TODO: from SDK

    s->frame.stVFrame.s16OffsetTop = 0;
    s->frame.stVFrame.s16OffsetBottom = 0;
    s->frame.stVFrame.s16OffsetLeft = 0;
    s->frame.stVFrame.s16OffsetRight = 0;

    //The next options i know nothing about, ill set them to the same values set by the sdk
    s->frame.stVFrame.u32TimeRef = 0;
    s->frame.stVFrame.u64PTS = 0;
    s->frame.stVFrame.pPrivateData = NULL;
    s->frame.stVFrame.u32FrameFlag = 0;

    //Set stride and size of each frame
    s->frame.stVFrame.u32Length[0] = y_size;
    s->frame.stVFrame.u32Length[1] = uv_size;
    s->frame.stVFrame.u32Length[2] = uv_size;

    s->frame.stVFrame.u32Stride[0] = av_image_get_linesize(avctx->pix_fmt, avctx->width, 0);
    s->frame.stVFrame.u32Stride[1] = av_image_get_linesize(avctx->pix_fmt, avctx->width, 1);
    s->frame.stVFrame.u32Stride[2] = av_image_get_linesize(avctx->pix_fmt, avctx->width, 2);

    //Set physical and vertual addresses of the frame
    s->frame.u32PoolId = CVI_VB_Handle2PoolId(s->enc_block);

    s->frame.stVFrame.u64PhyAddr[0] = CVI_VB_Handle2PhysAddr(s->enc_block);
    s->frame.stVFrame.pu8VirAddr[0] = (CVI_U8 *)CVI_SYS_Mmap(s->frame.stVFrame.u64PhyAddr[0], s->frame.stVFrame.u32Length[0]);
    memset(s->frame.stVFrame.pu8VirAddr[0], 0, s->frame.stVFrame.u32Length[0]);

    s->frame.stVFrame.u64PhyAddr[1] = ALIGN(s->frame.stVFrame.u64PhyAddr[0] + s->frame.stVFrame.u32Length[0], VENC_ALIGN_W);
    s->frame.stVFrame.pu8VirAddr[1] = (CVI_U8 *)CVI_SYS_Mmap(s->frame.stVFrame.u64PhyAddr[1], s->frame.stVFrame.u32Length[1]);
    memset(s->frame.stVFrame.pu8VirAddr[1], 0, s->frame.stVFrame.u32Length[1]);

    s->frame.stVFrame.u64PhyAddr[2] = ALIGN(s->frame.stVFrame.u64PhyAddr[1] + s->frame.stVFrame.u32Length[1], VENC_ALIGN_W);
    s->frame.stVFrame.pu8VirAddr[2] = (CVI_U8 *)CVI_SYS_Mmap(s->frame.stVFrame.u64PhyAddr[2], s->frame.stVFrame.u32Length[2]);
    memset(s->frame.stVFrame.pu8VirAddr[2], 0, s->frame.stVFrame.u32Length[2]);

    /*av_log(avctx, AV_LOG_ERROR, "Got addresses %16x %16x, %16x %16x, %16x %16x\n",
           s->frame.stVFrame.u64PhyAddr[0], s->frame.stVFrame.pu8VirAddr[0],
           s->frame.stVFrame.u64PhyAddr[1], s->frame.stVFrame.pu8VirAddr[1],
           s->frame.stVFrame.u64PhyAddr[2], s->frame.stVFrame.pu8VirAddr[2]
    );*/
    /*av_log(avctx, AV_LOG_ERROR, "Calculated: %08u %08u %08u - %08u %08u %08u\n",
           s->frame.stVFrame.u32Stride[0], s->frame.stVFrame.u32Stride[1], s->frame.stVFrame.u32Stride[2],
           s->frame.stVFrame.u32Length[0], s->frame.stVFrame.u32Length[1], s->frame.stVFrame.u32Length[2]
    );
    av_log(avctx, AV_LOG_ERROR, "Ffmpeg: %08u %08u %08u - %08u %08u %08u\n",
           frame->linesize[0], frame->linesize[1], frame->linesize[2],
           0, 0, 0
    );*/

    //Start the reception
    rec_param.s32RecvPicNum=-1;
    ret = CVI_VENC_StartRecvFrame(0, &rec_param);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "start recv frame, error %08x\n", (uint32_t) ret);
        return ret;
    }

    return 0;
}

static av_cold int sophgo_encode_init(AVCodecContext *avctx)
{
    SophgoCodecContext *s = avctx->priv_data;
    int ret = AVERROR_ENCODER_NOT_FOUND;
    //const char *role;

    s->avctx = avctx;

    s->enc_pool = VB_INVALID_HANDLE;
    s->enc_block = VB_INVALID_HANDLE;

    switch (avctx->codec->id) {
    case AV_CODEC_ID_H264:
        //only h264 is corrently supported
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Codec not supported!\n");
        return AVERROR(ENOSYS);
    }

    //Try to initialize the hardware
    ret = sophgo_venc_init(s);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Error initializing channel\n");
        goto fail;
    }

    //Setup VENC frame
    //ret = sophgo_setup_frame(s, frame);
    ret = sophgo_setup_frame(s);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "unable to setup frame! %08x\n", ret);
        return ret;
    }

    //s->receive_ready = false;

    av_log(avctx, AV_LOG_INFO, "init done\n");

    return 0;
fail:
    return ret;
}


static int sophgo_encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                            const AVFrame *frame, int *got_packet)
{
    SophgoCodecContext *s = avctx->priv_data;
    int ret;
    //unsigned int framesize;
    unsigned int off_temp;
    off_temp = 0;
    VENC_CHN_STATUS_S stStat;
    VENC_STREAM_S stStream;

    if (!frame) {
        av_log(avctx, AV_LOG_WARNING, "no frame recieved!\n");
        return 0;
    }

    /*if (!s->receive_ready) {
        //Setup frame and allocate buffer and start receiving
        ret = sophgo_setup_frame(s, frame);
        if (ret != 0) {
            av_log(avctx, AV_LOG_ERROR, "unable to setup frame! %08x\n", ret);
            return ret;
        }
        s->receive_ready = true;
    }*/

    //if (framesize != s->frame_total_size)
    //av_log(avctx, AV_LOG_WARNING, "Buffer sizes differ, ffmpeg calculated %u but private calc was %u !\n", framesize, s->frame_total_size);

    memcpy(s->frame.stVFrame.pu8VirAddr[0], &frame->data[0][0], s->frame.stVFrame.u32Length[0]);
    memcpy(s->frame.stVFrame.pu8VirAddr[1], &frame->data[1][0], s->frame.stVFrame.u32Length[1]);
    memcpy(s->frame.stVFrame.pu8VirAddr[2], &frame->data[2][0], s->frame.stVFrame.u32Length[2]);

    ret = CVI_VENC_SendFrame(0, &s->frame, 20000);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "send frame, error %08x\n", (uint32_t) ret);
        return ret;
    }

    ret = CVI_VENC_QueryStatus(0, &stStat);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "query, error %08x\n", (uint32_t) ret);
        return ret;
    }
    if (!stStat.u32CurPacks) {
        av_log(avctx, AV_LOG_INFO, "no packs\n");
        return 0;
    }

    stStream.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);

    ret = CVI_VENC_GetStream(0, &stStream, -1);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "getstream frame, error %08x\n", (uint32_t) ret);
        return ret;
    }

    unsigned int total_len;

    VENC_PACK_S *ppack;
    for (unsigned int i=0; i<stStream.u32PackCount; i++) {
        ppack = &stStream.pstPack[i];

        total_len += (ppack->u32Len - ppack->u32Offset);
    }

    //Now, allocate the packet and write it
    ret = ff_alloc_packet(avctx, pkt, total_len);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "unable to allocate packet with size %u\n", (uint32_t) total_len);
        return ret;
    }

    off_temp = 0;
    for (unsigned int i=0; i<stStream.u32PackCount; i++) {
        ppack = &stStream.pstPack[i];

        memcpy(pkt->data + off_temp, ppack->pu8Addr + ppack->u32Offset, ppack->u32Len - ppack->u32Offset);
        off_temp += ppack->u32Len - ppack->u32Offset;

        //total_len += (ppack->u32Len - ppack->u32Offset);
        /*fwrite(ppack->pu8Addr + ppack->u32Offset,
            ppack->u32Len - ppack->u32Offset, 1, out);*/
    }

    *got_packet=1;

    CVI_VENC_ReleaseStream(0, &stStream);
    free(stStream.pstPack);

    return 0;
}

static av_cold int sophgo_encode_end(AVCodecContext *avctx)
{
    SophgoCodecContext *s = avctx->priv_data;

    CVI_VENC_StopRecvFrame(0);
    CVI_VENC_ResetChn(0);
    CVI_VENC_DestroyChn(0);

    for (int i=0; i<3; i++) {
        if (s->frame.stVFrame.pu8VirAddr[i] != 0)
            CVI_SYS_Munmap(s->frame.stVFrame.pu8VirAddr[i], s->frame.stVFrame.u32Length[i]);
    }

    if (s->enc_block != 0)
        CVI_VB_ReleaseBlock(s->enc_block);
    if (s->enc_pool != 0)
        CVI_VB_DestroyPool(s->enc_pool);

    s->enc_block = s->enc_pool = VB_INVALID_HANDLE;

    return 0;
}

#define OFFSET(x) offsetof(SophgoCodecContext, x)
#define VDE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_ENCODING_PARAM
#define VE  AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "zerocopy", "Try to avoid copying input frames if possible", OFFSET(input_zerocopy), AV_OPT_TYPE_INT, { .i64 = CONFIG_OMX_RPI }, 0, 1, VE },
    { "profile",  "Set the encoding profile", OFFSET(profile), AV_OPT_TYPE_INT,   { .i64 = FF_PROFILE_UNKNOWN },       FF_PROFILE_UNKNOWN, FF_PROFILE_H264_HIGH, VE, "profile" },
    { "baseline", "",                         0,               AV_OPT_TYPE_CONST, { .i64 = FF_PROFILE_H264_BASELINE }, 0, 0, VE, "profile" },
    { "main",     "",                         0,               AV_OPT_TYPE_CONST, { .i64 = FF_PROFILE_H264_MAIN },     0, 0, VE, "profile" },
    { "high",     "",                         0,               AV_OPT_TYPE_CONST, { .i64 = FF_PROFILE_H264_HIGH },     0, 0, VE, "profile" },
    { NULL }
};

static const enum AVPixelFormat sophgo_encoder_pix_fmts[] = {
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_NONE
    //AV_PIX_FMT_YUV422P, AV_PIX_FMT_NONE
};

static const AVClass sophgo_h264enc_class = {
    .class_name = "h264_sophgo",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};
const FFCodec ff_h264_sophgo_encoder = {
    .p.name           = "h264_sophgo",
    .p.long_name      = NULL_IF_CONFIG_SMALL("Sophgo CV1800B/SG2002 H.264 video encoder"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_H264,
    .priv_data_size   = sizeof(SophgoCodecContext),
    .init             = sophgo_encode_init,
    FF_CODEC_ENCODE_CB(sophgo_encode_frame),
    .close            = sophgo_encode_end,
    .p.pix_fmts       = sophgo_encoder_pix_fmts,
    .p.capabilities   = AV_CODEC_CAP_DELAY,
    .caps_internal    = FF_CODEC_CAP_INIT_CLEANUP,
    .p.priv_class     = &sophgo_h264enc_class,
};
