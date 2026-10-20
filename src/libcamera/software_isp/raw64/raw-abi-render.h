/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once
#include "raw-abi-engine-generated.h"
#include "raw-abi-colour-generated.h"
#include "raw-abi-protocol.h"
#include <pthread.h>

namespace rawabi {
template<class Engine>
void render(Engine &engine, const Request &q, const uint8_t *input, uint8_t *output)
{
    engine.window_.width = q.width;
    engine.blue_ = q.blue; engine.green_ = q.green; engine.red_ = q.red;
    for (unsigned i=0; i<256; ++i) {
        engine.bluePacked_[i] = uint32_t(q.blue[i]) | 0xff000000U;
        engine.greenPacked_[i] = uint32_t(q.green[i]) << 8;
        engine.redPacked_[i] = uint32_t(q.red[i]) << 16;
    }
    struct Job { Engine *engine; const Request *q; const uint8_t *input; uint8_t *output; unsigned first,last; };
    std::array<Job,4> jobs;
    const auto work = [](void *opaque) -> void * {
        auto &j = *static_cast<Job *>(opaque); const auto &r = *j.q;
        const auto row = [&](unsigned y, uint8_t *dst) {
            const uint8_t *rows[] = {
                j.input + size_t(y ? y-1 : 1) * r.inputStride + r.inputOffset,
                j.input + size_t(y) * r.inputStride + r.inputOffset,
                j.input + size_t(y+1 < r.height ? y+1 : r.height-2) * r.inputStride + r.inputOffset
            };
            switch (r.pattern ^ (y & 1)) {
            case 0: j.engine->template debayer10P_BGBG_BGR888<true,false>(dst,rows); break;
            case 1: j.engine->template debayer10P_GRGR_BGR888<true,false>(dst,rows); break;
            case 2: j.engine->template debayer10P_GBGB_BGR888<true,false>(dst,rows); break;
            case 3: j.engine->template debayer10P_RGRG_BGR888<true,false>(dst,rows); break;
            }
        };
        if (r.format == 1) {
            std::array<uint8_t,4160*4*2> rgb;
            uint8_t *uv = j.output + size_t(r.height) * r.outputStride;
            for (unsigned y=j.first; y<j.last; y+=2) {
                row(y,rgb.data()); row(y+1,rgb.data()+r.width*4);
                colourToNv12Fused(rgb.data(),r.width*4,
                    j.output+size_t(y)*r.outputStride,r.outputStride,
                    uv+size_t(y/2)*r.outputStride,r.outputStride,r.width,2);
            }
        } else {
            for (unsigned y=j.first; y<j.last; ++y)
                row(y,j.output+size_t(y)*r.outputStride);
        }
        return nullptr;
    };
    std::array<pthread_t,3> workers;
    std::array<bool,3> started{};
    for (unsigned i=0; i<4; ++i)
        jobs[i] = {&engine,&q,input,output,2*(i*(q.height/2)/4),2*((i+1)*(q.height/2)/4)};
    for (unsigned i=0; i<3; ++i) {
        if (!pthread_create(&workers[i],nullptr,work,&jobs[i])) started[i] = true;
        else work(&jobs[i]);
    }
    work(&jobs[3]);
    for (unsigned i=0; i<3; ++i) if (started[i]) pthread_join(workers[i],nullptr);
}
}
