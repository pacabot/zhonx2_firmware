#include "fw_protocol.h"
#include "fw_layout.h"
#include <string.h>
int fw_protocol(const fw_flash_t *f, fw_update_t *u, const fw_packet_t *p, fw_reply_t *r)
{
    int status=-1, boot=0;
    uint32_t n=p->word[5];
    if (p->word[0]==FW_PACKET_MAGIC && p->word[6]==1 && n<=1024 &&
        fw_crc32_more(fw_crc32(p,28),p->payload,n)==p->word[7]) {
        switch (p->word[1]) {
        case FW_INFO: if (!n) status=0; break;
        case FW_BEGIN: if (!n) status=fw_update_begin(f,u,p->word[3],p->word[4]); break;
        case FW_DATA: status=fw_update_write(f,u,p->word[3],p->payload,n); break;
        case FW_COMMIT:
            if (!n) {
                status=fw_update_seal(f,u);
                if (!status) status=fw_update_install(f);
            }
            break;
        case FW_BOOT:
            if (!n && !u->active) { status=fw_application_valid(f); boot=!status; }
            break;
        default: break;
        }
    }
    memset(r,0,sizeof *r);
    r->word[0]=FW_PACKET_MAGIC; r->word[1]=p->word[2]; r->word[2]=(uint32_t)status;
    r->word[3]=FW_IMAGE_MAX; r->word[4]=u->received;
    r->word[5]=FW_APP_BASE; r->word[6]=1; r->word[7]=fw_crc32(r,28);
    return boot;
}
