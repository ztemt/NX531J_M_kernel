/*
**
**  Project: multi channels module
**  File:      multi_channel_hook.c
**  Author: huangjunyuan
**  Date:    09/6/2015
**
**  Purpose:
**      multi channels module.
**
**  History:
**  <author>   <time>          <version >   <desc>
*/
#include <net/ip.h>
#include <linux/init.h>
#include <net/tcp.h>
#include <net/sock.h>
#include <linux/time.h>
#include <linux/init.h>
#include <net/netlink.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/file.h>
#include <linux/icmp.h>
#include <net/icmp.h>
#include <linux/netfilter.h>
#include <linux/rtnetlink.h>
#include <linux/netfilter/xt_state.h>
#include <net/netfilter/ipv4/nf_reject.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/nf_conntrack_tuple.h>
#include <net/netfilter/nf_conntrack_ecache.h>
#include <linux/netfilter/nf_conntrack_common.h>

#ifdef CONFIG_SYSCTL
#include <linux/sysctl.h>
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("huangjunyuan0016003461@nubia.com.cn");
MODULE_DESCRIPTION("multi channels module");

#define K1S                         (HZ)        // ~1s
#define WIFI_INDEX                  0
#define LTE_INDEX                   2
#define CONGESTION_FLAG             3
#define IDLE_FLAG                   3
#define IDLE_BACK_RTT               1
#define IF_NUM                      4
#define DEV_NAME_LEA                16
#define MIN_WEIGHT                  5           //wifi min weight
#define MAX_WEIGHT                  90           //wifi max weight
#define MIN_SPEED_LEVEL             10          //speed + rtt min 10KB/s
#define SHAKE_SPEED_LEVEL           50          //speed back standard
#define MAX_SPEED_LEVEL             100          //calc weight by speed small limte
#define SYN_RTT_GOOD_LEVEL          300          //tcp rtt > 300ms
#define SYN_RTT_LEVEL               500          //tcp rtt > 500ms
#define NETLINK_MULTI_CHANNEL       29
#define UUID_NUM                    256
#define DEFAULT_SYN_RETRANSMIT_RTT  1000         //default retrasmit rtt = 1000ms

#define MULTI_WAN1 "wlan0"
#define MULTI_WAN2 "wlan1"
#define MULTI_WAN3 "rmnet_data0"
#define MULTI_WAN4 "eth0"

#ifdef MULTI_DEBUG
    #define MULTI_PRINT(args...) \
    do{ \
        printk(args); \
    }while(0)
#else
    #define MULTI_PRINT(args...) do{}while(0)
#endif

static int multi_channel_enable;
static unsigned int multi_weight[IF_NUM];
static struct timeval last_time;
static unsigned int multi_rtt[IF_NUM][5];
static struct timer_list multi_channel_timer;
static unsigned int speedtest_uid;
static struct sock *multi_channel_sock = NULL;
static unsigned int forbid_wifi_uuid[UUID_NUM];
static unsigned int forbid_lte_uuid[UUID_NUM];
static char wan_dev[IF_NUM][DEV_NAME_LEA] = {MULTI_WAN1,MULTI_WAN2,MULTI_WAN3,MULTI_WAN4};


enum {
    NET_2G = 2,
    NET_3G = 3,
    NET_4G = 4,
};

enum {
    WIFI_WISP =0,
    WIFI_AP = 1,
};

enum {
    IF_DOWN =0,
    IF_UP = 1,
};


enum {
    WIFI_AUTHED = 0,
    WIFI_UNAUTHED = 1,
};

enum {
    DISABLE = 0,
    ENABLE = 1,
};

enum {
    MULTI_DROP = 0,
    MULTI_MARKED = 1,
    MULTI_ACEEPT = 2,
};

enum {
    SET_MULTI_CHANNEL_ENABLE=0x13,
    SET_MULTI_CHANNEL_DISABLE = 0x14,
    SET_MULTI_CHANNEL_IF_UP,
    SET_MULTI_CHANNEL_IF_DOWN,
    SET_MULTI_CHANNEL_LTE_MODE,
    SET_MULTI_CHANNEL_WIFI_MODE,
    SET_MULTI_CHANNEL_WIFI_AUTH,
    SET_MULTI_CHANNEL_WIFI_UUID,
    SET_MULTI_CHANNEL_LTE_UUID,
    SET_MULTI_CHANNEL_WIFI_SIGNAL,
};


struct inface_information{
    unsigned int if_up;
    unsigned int net_mode;
    unsigned int wifi_mode;
    unsigned int wifi_auth;
    unsigned int max_speed;
    unsigned int alive_speed;
    unsigned int idle_rtt;
    unsigned int congestion_flag;
    unsigned long long last_bytes;
};


static struct inface_information if_info[IF_NUM];
extern int (*multi_calc_rtt)(struct sock *sk, struct sk_buff *skb);

DEFINE_RWLOCK(multi_channel_lock);
#define multi_read_lock()       read_lock_bh(&multi_channel_lock);
#define multi_read_unlock()     read_unlock_bh(&multi_channel_lock);
#define multi_write_lock()      write_lock_bh(&multi_channel_lock);
#define multi_write_unlock()    write_unlock_bh(&multi_channel_lock);

static int mark_https_skb(struct sk_buff *skb)
{

    unsigned short dport;
    struct tcphdr *tcph;
    struct iphdr *iph;

    iph = ip_hdr(skb);
    if(iph && iph->protocol == IPPROTO_TCP){

        tcph = (struct tcphdr*)((char *)iph + iph->ihl*4);
        #ifdef __LITTLE_ENDIAN
        dport = ntohs(tcph->dest);
        #else
        dport = tcph->dest;
        #endif

        if(dport == 443){
            if(if_info[0].if_up){
                skb->mark = 0x0100 & 0xFFFF;
            }
            else if(if_info[1].if_up){
                skb->mark = 0x0200 & 0xFFFF;
            }
            else if(if_info[2].if_up){
                skb->mark = 0x0300 & 0xFFFF;
            }
            else if(if_info[3].if_up){
                skb->mark = 0x0400 & 0xFFFF;
            }
            return MULTI_MARKED;
        }
    }
    return MULTI_ACEEPT;
}

static int mark_speedtest_uuid_skb(unsigned int uuid,struct sk_buff *skb)
{
    static int speedtest_mark[IF_NUM-1]={0};

    if(speedtest_uid && uuid == speedtest_uid){
        if(!speedtest_mark[2] && if_info[3].if_up){
            speedtest_mark[2] = 1;
            skb->mark = 0x0400 & 0xFFFF;
        }
        else if(!speedtest_mark[1] && if_info[2].if_up){
            speedtest_mark[1] = 1;
            skb->mark = 0x0300 & 0xFFFF;
        }
        else if(!speedtest_mark[0] && if_info[1].if_up){
            speedtest_mark[0] = 1;
            skb->mark = 0x0200 & 0xFFFF;
        }
        else{
            speedtest_mark[0] = 0;
            speedtest_mark[1] = 0;
            speedtest_mark[2] = 0;
            if(if_info[0].if_up){
                skb->mark = 0x0100 & 0xFFFF;
            }
        }
        return MULTI_MARKED;
    }
    return MULTI_ACEEPT;
}

static int mark_nubia_firewall_uuid(unsigned int uuid, struct sk_buff *skb)
{
    int i = 0;
    int j = 0;
    int lte_mark;
    int ret = MULTI_ACEEPT;

    lte_mark = 0;
    multi_read_lock();
    for(i = 0; i < UUID_NUM; i++){
        if(forbid_wifi_uuid[i]){
            if(uuid == forbid_wifi_uuid[i]){
                if(if_info[LTE_INDEX].if_up){
                    skb->mark = 0x0300 & 0xFFFF;
                    lte_mark = 1;
                    ret = MULTI_MARKED;
                }
                else{
                    ret = MULTI_DROP;
                }
                break;
            }
        }
        else{
            break;
        }
    }

    for(j = 0; j < UUID_NUM; j++){
        if(forbid_lte_uuid[j]){
            if(uuid == forbid_lte_uuid[j]){
                if(lte_mark || !if_info[WIFI_INDEX].if_up){
                    ret = MULTI_DROP;
                }
                else{
                    skb->mark = 0x0100 & 0xFFFF;
                    ret = MULTI_MARKED;
                }
                break;
            }
        }
        else{
            break;
        }
    }
    multi_read_unlock();

    return ret;
}

static int mark_special_uuid(unsigned int hook,struct sk_buff *skb)
{
    struct file *filp;
    unsigned int uuid;
    int ret = MULTI_ACEEPT;
    int ret1 = MULTI_ACEEPT;

    if (hook == NF_INET_LOCAL_OUT &&
        skb->sk != NULL &&
        skb->sk->sk_socket != NULL &&
        skb->sk->sk_socket->file != NULL){

        filp = skb->sk->sk_socket->file;
        uuid = filp->f_cred->fsuid.val;

        ret = mark_speedtest_uuid_skb(uuid,skb);

        ret1 = mark_nubia_firewall_uuid(uuid,skb);
        if(MULTI_DROP == ret1){
            goto multi_drop;
        }
        else if(MULTI_MARKED == ret1){
            ret = MULTI_MARKED;
        }

        if( uuid == 1000){
            skb->mark = 0x0 & 0xFFFF;
            ret = MULTI_MARKED;
        }
    }

    if(MULTI_ACEEPT == ret){
        ret = mark_https_skb(skb);
    }

    return ret;

multi_drop:
    nf_send_unreach(skb, ICMP_PORT_UNREACH);
    return MULTI_DROP;
}

static void set_syn_rtt_by_ct_mark(unsigned int ct_mark,int rtt)
{
    int i = -1;
    int j = 0;
    static int index[IF_NUM];

    if(0x0100  == ct_mark){
        i = 0;
    }
    else if(0x0200 == ct_mark){
        i = 1;
    }
    else if(0x0300 == ct_mark){
        i = 2;
    }
    else if(0x0400 == ct_mark){
        i = 3;
    }

    if(-1 != i){
        j = index[i];
        if(4 == index[i]){
            index[i] = 0;
        }
        else{
            index[i] += 1;
        }
        multi_rtt[i][j] = rtt;
    }
    return;
}

static int mark_retransmit_skb(unsigned int hook,struct nf_conn *ct, struct sk_buff *skb)
{
    int ret = MULTI_ACEEPT;
    u_int32_t ct_mark = ct->mark & 0xFFFF;
    if(ct_mark == 0x0100 || ct_mark == 0x0200 ||
            ct_mark == 0x0300 || ct_mark== 0x0400){
        skb->mark = ct_mark;
        ret = MULTI_MARKED;
        set_syn_rtt_by_ct_mark(ct_mark,DEFAULT_SYN_RETRANSMIT_RTT);
    }
    return ret;
}

static u_int32_t multi_channel_hook(const struct nf_hook_ops *ops,
             struct sk_buff *skb,
             const struct net_device *in,
             const struct net_device *out,
             int (*okfn)(struct sk_buff *))
{
    int ret = MULTI_ACEEPT;
    struct nf_conn *ct = NULL;
    enum ip_conntrack_info ctinfo;
    unsigned int state_mask = 0;
    unsigned int hook = ops->hooknum;

    if(DISABLE == multi_channel_enable){
        goto OUT_DONE;
    }

    ct = nf_ct_get(skb, &ctinfo);
    if(NULL == ct){
        goto OUT_DONE;
    }

    state_mask = XT_STATE_BIT(ctinfo);
    if((state_mask & XT_STATE_BIT(IP_CT_ESTABLISHED)) ||
        (state_mask & XT_STATE_BIT(IP_CT_RELATED))){

        skb->mark = ct->mark & 0xFFFF;

    }else if(IP_CT_NEW == ctinfo){

        if(MULTI_ACEEPT == mark_retransmit_skb(hook,ct,skb)){
            ret = mark_special_uuid(hook,skb);
            if(MULTI_ACEEPT == ret){
                if((prandom_u32() & 0x7FFFFFFF) < (0x147ae14 * multi_weight[0])){
                    skb->mark = 0x0100 & 0xFFFF;
                }
                else if((prandom_u32() & 0x7FFFFFFF) < (0x147ae14 * multi_weight[1])){
                    skb->mark = 0x0200 & 0xFFFF;
                }
                else if((prandom_u32() & 0x7FFFFFFF) < (0x147ae14 * multi_weight[2])){
                    skb->mark = 0x0300 & 0xFFFF;
                }
                else if((prandom_u32() & 0x7FFFFFFF) < (0x147ae14 * multi_weight[3])){
                    skb->mark = 0x0400 & 0xFFFF;
                }
            }
            else if(MULTI_DROP == ret){
                return NF_DROP;
            }
        }
        ct->mark = skb->mark;
    }
    else{
        goto OUT_DONE;
    }

    if(NF_INET_LOCAL_OUT == hook){
        if (ip_route_me_harder(skb, RTN_UNSPEC)){
            return NF_DROP;
        }
    }
OUT_DONE:
    return NF_ACCEPT;
}


static struct nf_hook_ops multi_channel_ops[] __read_mostly = {
    {
        .hook       = multi_channel_hook,
        .owner      = THIS_MODULE,
        .pf         = PF_INET,
        .hooknum    = NF_INET_LOCAL_OUT,
        .priority   = NF_IP_PRI_CONNTRACK + 1,
    },

    {
        .hook       = multi_channel_hook,
        .owner      = THIS_MODULE,
        .pf         = PF_INET,
        .hooknum    = NF_INET_PRE_ROUTING,
        .priority   = NF_IP_PRI_CONNTRACK + 1,
    },

};

static int calc_tcp_syn_rtt(struct sock *sk, struct sk_buff *skb)
{
    int tmp_rtt = 0;
    u_int32_t ct_mark = 0x0;
    struct nf_conn *ct = NULL;
    enum ip_conntrack_info ctinfo;
    struct tcp_sock *tp = tcp_sk(sk);
    struct inet_connection_sock *icsk = inet_csk(sk);

    if(TCP_SYN_SENT == sk->sk_state){
        ct = nf_ct_get(skb, &ctinfo);
        if(ct != NULL && !icsk->icsk_retransmits){
            tmp_rtt =  tcp_time_stamp - tp->retrans_stamp;
            if(tmp_rtt){
                ct_mark = ct->mark & 0xFFFF;
                tmp_rtt = jiffies_to_msecs(tmp_rtt);
                set_syn_rtt_by_ct_mark(ct_mark,tmp_rtt);
            }
        }
    }
    return 0;
}

static int dev_isalive(const struct net_device *dev)
{
    return dev->reg_state <= NETREG_REGISTERED;
}

static int cal_dev_speed(void)
{
    int i=0;
    unsigned int tmp_speed;
    struct net_device *dev;
    const struct rtnl_link_stats64 *stats;
    struct rtnl_link_stats64 temp;

    for(i=0;i<IF_NUM;i++) {
         if(NET_2G != if_info[i].net_mode &&
            WIFI_AP != if_info[i].wifi_mode && if_info[i].if_up){
            dev = dev_get_by_name(&init_net, wan_dev[i]);
              if(dev) {
                 if (dev_isalive(dev) && (stats = dev_get_stats(dev, &temp))){
                    if(0 == if_info[i].last_bytes){
                        if_info[i].last_bytes = stats->rx_bytes;
                    }
                    else{
                        if(stats->rx_bytes < if_info[i].last_bytes){
                            tmp_speed = stats->rx_bytes/1024;
                        }else{
                            tmp_speed = (stats->rx_bytes - if_info[i].last_bytes)/1024;
                        }

                        if_info[i].last_bytes= stats->rx_bytes;

                        if(tmp_speed > if_info[i].max_speed){
                            if_info[i].max_speed = tmp_speed;
                        }
                        else if(if_info[i].congestion_flag >= CONGESTION_FLAG){
                            if((tmp_speed + SHAKE_SPEED_LEVEL) <= if_info[i].max_speed){
                                if_info[i].max_speed = if_info[i].max_speed/2;
                                if(tmp_speed > if_info[i].max_speed){
                                    if_info[i].max_speed = tmp_speed;
                                }
                            }
                            if_info[i].congestion_flag = 0;
                            if_info[i].idle_rtt = 0;
                        }

                        if_info[i].alive_speed = if_info[i].max_speed - tmp_speed;

                        MULTI_PRINT("func = %s,dev_name = %s,max speed is = %u, alive speed = %u\n",
                                    __func__,wan_dev[i],if_info[i].max_speed, if_info[i].alive_speed);
                    }
                 }
                 dev_put(dev);
              }
         }
    }

    return 0;
}

static void calc_weight_by_default(void)
{
    int i = 0,j = 0;
    int sum[IF_NUM] = {0};

    for(j = 0;j < IF_NUM; j++){
        if(NET_2G != if_info[j].net_mode &&
            WIFI_AP != if_info[j].wifi_mode &&
            if_info[j].if_up){
            for(i = j; i < IF_NUM; i++){
                if(NET_2G != if_info[i].net_mode &&
                    WIFI_AP != if_info[i].wifi_mode &&
                    if_info[i].if_up){
                    sum[j] += 1;
                }
            }
            if(sum[j]){
                multi_weight[j] = 100/sum[j];
            }
        }
        else{
            multi_weight[j] = 0;
        }
    }
    return;
}

static void statistics_if_rtt(int index)
{
    int j = index;
    int rtt;

    if(multi_rtt[j][0] && multi_rtt[j][1] && multi_rtt[j][2]
        && multi_rtt[j][3] && multi_rtt[j][4]){

        rtt = (multi_rtt[j][0] + multi_rtt[j][1] + multi_rtt[j][2] + multi_rtt[j][3] + multi_rtt[j][4])/5;
        MULTI_PRINT("func = %s,dev name = %s,rtt = %d\n",__func__,wan_dev[j],rtt);
        if(rtt >= SYN_RTT_LEVEL){
            if(if_info[j].congestion_flag < CONGESTION_FLAG){
                if_info[j].congestion_flag += 1;
            }
            if(if_info[j].idle_rtt){
                if_info[j].idle_rtt -= 1;
            }
            memset(multi_rtt[j],0x0,5*sizeof(unsigned int));
        }
        else{
            if(if_info[j].congestion_flag){
                if_info[j].congestion_flag -= 1;
            }

            if(if_info[j].idle_rtt < IDLE_FLAG){
                if(rtt <= SYN_RTT_GOOD_LEVEL){
                    if_info[j].idle_rtt += 1;
                }
                else if(if_info[j].idle_rtt){
                    if_info[j].idle_rtt -= 1;
                }
            }
        }
    }
    return;
}

static int calc_weight_by_speed(void)
{
    int i = 0,j = 0;
    unsigned int sum_speed[IF_NUM] = {0};
    int cal_speed_weight = 1;
    int calc_by_speed_flag = 0;
    unsigned int tmp_weight[IF_NUM];

    for(j = 0;j < IF_NUM; j++){
        if(NET_2G != if_info[j].net_mode &&
            WIFI_AP != if_info[j].wifi_mode && if_info[j].if_up){

            statistics_if_rtt(j);

            MULTI_PRINT("func = %s,dev_name[%d] = %s ,congestion_flag = %d,idle_rtt = %d\n",
                        __func__,j,wan_dev[j],if_info[j].congestion_flag,if_info[j].idle_rtt);

            if(if_info[j].max_speed >= MIN_SPEED_LEVEL){
               if(if_info[j].max_speed < MAX_SPEED_LEVEL){
                    if(if_info[j].idle_rtt > IDLE_BACK_RTT && !calc_by_speed_flag){
                        cal_speed_weight = 0;
                    }
                    else{
                        calc_by_speed_flag = 1;
                    }
                }
            }
            else{
                cal_speed_weight = 0;
                return cal_speed_weight;
            }

            for(i = j; i < IF_NUM; i++){
                if(if_info[i].if_up &&
                    NET_2G != if_info[i].net_mode &&
                    WIFI_AP != if_info[i].wifi_mode){
                    sum_speed[j] += (20*if_info[i].alive_speed + 80*if_info[i].max_speed)/100;
                }
            }
            if(sum_speed[j]){
                tmp_weight[j] = (20*if_info[j].alive_speed + 80*if_info[j].max_speed)/sum_speed[j];
                if(WIFI_INDEX == j){
                    if(tmp_weight[j] < MIN_WEIGHT &&
                        !if_info[j].congestion_flag){
                        tmp_weight[j] = MIN_WEIGHT;
                    }
                    else if(tmp_weight[j] > MAX_WEIGHT &&
                            if_info[LTE_INDEX].if_up &&
                            NET_2G != if_info[LTE_INDEX].net_mode){
                        tmp_weight[j] = MAX_WEIGHT;
                    }
                }
            }
        }
        else{
            if_info[j].max_speed = 0;
            tmp_weight[j] = 0;
        }
    }

    if(cal_speed_weight){
        memcpy(multi_weight,tmp_weight,sizeof(unsigned int) * IF_NUM);
    }
    return cal_speed_weight;
}

static int cal_dev_weight(void)
{
#ifdef MULTI_DEBUG
    int i = 0;
#endif
    int cal_speed_weight = 0;
    struct timeval tv_now;

    do_gettimeofday(&tv_now);
    if((tv_now.tv_sec - last_time.tv_sec) >= 5){

        last_time = tv_now;
        cal_speed_weight = calc_weight_by_speed();

        if(!cal_speed_weight){
            calc_weight_by_default();
        }
#ifdef MULTI_DEBUG
        for(i = 0; i < IF_NUM; i++){
            MULTI_PRINT("func = %s,weight[%d] = %d\n",__func__, i,multi_weight[i]);
        }
#endif
    }
    return 0;
}

static int check_if_status(void)
{
    int i = 0;
    for(i = 0; i< IF_NUM; i++){
        if(WIFI_UNAUTHED == if_info[i].wifi_auth && if_info[i].if_up){
            memset(multi_weight,0x0,sizeof(multi_weight));
            multi_weight[i] = 100;
            MULTI_PRINT("func = %s,multi_weight[%d]= %d\n",__func__, i, multi_weight[i]);
            return 1;
        }
    }
    return 0;
}

void get_timer_statistic(void)
{
    if(check_if_status()){
        cal_dev_speed();
        return;
    }
    cal_dev_speed();
    cal_dev_weight();
    return;
}


void multi_channel_timer_expire(void)
{
    get_timer_statistic();

    mod_timer(&multi_channel_timer, jiffies + K1S);
}

void multi_channel_timer_start(void)
{

    struct timeval tv_now;
    init_timer(&multi_channel_timer);
    multi_channel_timer.function = (void*)multi_channel_timer_expire;
    multi_channel_timer.expires = jiffies +  K1S;
    do_gettimeofday(&tv_now);
    last_time = tv_now;
    add_timer (&multi_channel_timer);
}

static int change_ifname_to_index(char  *ifname)
{
    int i = -1;
    if(!memcmp(ifname,MULTI_WAN1,strlen(MULTI_WAN1))){
        i = 0;
    }
    else if(!memcmp(ifname,MULTI_WAN2,strlen(MULTI_WAN2))){
        i = 1;
    }
    else if(!memcmp(ifname,MULTI_WAN3,strlen(MULTI_WAN3))){
        i = 2;
    }
    else if(!memcmp(ifname,MULTI_WAN4,strlen(MULTI_WAN4))){
        i = 3;
    }
    return i;
}

static void init_changed_if_info(struct inface_information *info)
{
    info->max_speed = 0;
    info->alive_speed = 0;
    info->congestion_flag = 0;
    info->idle_rtt = IDLE_FLAG;
    return;
}

static int set_multi_channel_enable(struct nlmsghdr *nlh)
{
    u_int32_t *p = NULL;
    p = (u_int32_t *)NLMSG_DATA(nlh);
    speedtest_uid = *p;
    multi_channel_enable = ENABLE;
    printk("fucn = %s,speedtest uid = %u\n",__func__,speedtest_uid);
    return 0;
}

static int set_multi_channel_ifup(struct nlmsghdr *nlh)
{
    char *p = NULL;
    int i = 0;

    p = NLMSG_DATA(nlh);
    printk("func = %s,ifname = %s\n",__func__,p);

    if((i = change_ifname_to_index(p)) >= 0){
        if_info[i].if_up= IF_UP;
    }

    return 0;
}

static int set_multi_channel_ifdown(struct nlmsghdr *nlh)
{
    char *p = NULL;
    int i = 0;

    p = NLMSG_DATA(nlh);
    printk("func = %s,ifname = %s\n",__func__,p);

    if((i = change_ifname_to_index(p)) >= 0){
        init_changed_if_info(&if_info[i]);
        if_info[i].if_up = IF_DOWN;
        memset(multi_rtt[i],0x0,5*sizeof(unsigned int));
    }

    return 0;
}

static int set_multi_channel_lte_mode(struct nlmsghdr *nlh)
{
    char *p = NULL;
    u_int32_t  *q = NULL;
    int i = 0;

    p = NLMSG_DATA(nlh);
    q = (u_int32_t  *)(p+DEV_NAME_LEA);
    printk("func = %s,ifname = %s,lte mode = %u\n",__func__,p,*q);
    if((i = change_ifname_to_index(p)) >= 0){
        if(if_info[i].net_mode != *q){
            if_info[i].net_mode= *q;
            init_changed_if_info(&if_info[i]);
            memset(multi_rtt[i],0x0,5*sizeof(unsigned int));
        }
    }

    return 0;
}

static int set_multi_channel_wifi_mode(struct nlmsghdr *nlh)
{
    char *p = NULL;
    u_int32_t  *q = NULL;
    int i = 0;

    p = NLMSG_DATA(nlh);
    q = (u_int32_t  *)(p+DEV_NAME_LEA);
    printk("func = %s,ifname = %s,wifi mode = %u\n",__func__,p,*q);
    if((i = change_ifname_to_index(p)) >= 0){
        if(if_info[i].net_mode != *q){
            if_info[i].wifi_mode= *q;
            if(WIFI_AP == if_info[i].wifi_mode){
                init_changed_if_info(&if_info[i]);
                memset(multi_rtt[i],0x0,5*sizeof(unsigned int));
            }
        }
    }

    return 0;
}

static int set_multi_channel_wifi_auth(struct nlmsghdr *nlh)
{
    char *p = NULL;
    u_int32_t  *q = NULL;
    int i = 0;

    p = NLMSG_DATA(nlh);
    q = (u_int32_t  *)(p+DEV_NAME_LEA);
    printk("func = %s,ifname = %s,wifi auth state = %u\n",__func__,p,*q);
    if((i = change_ifname_to_index(p)) >= 0){
        if_info[i].wifi_auth= *q;
    }

    return 0;
}

static int set_multi_channel_uuid(struct nlmsghdr *nlh,int msg_type)
{
    char *p = NULL;
    int *q = NULL;
    int count = 0;
    q = (int *)NLMSG_DATA(nlh);
    p =(char *)NLMSG_DATA(nlh) + sizeof(int);

    count = *q;
    printk("func = %s,msy type = %u,uuid count = %d\n",__func__,msg_type,count);

    multi_write_lock();
    if(SET_MULTI_CHANNEL_WIFI_UUID == msg_type){
        memset(&forbid_wifi_uuid,0x0,UUID_NUM*sizeof(u_int32_t));
        if(count){
            memcpy(&forbid_wifi_uuid,p,count*sizeof(u_int32_t));
        }
    }
    else if(SET_MULTI_CHANNEL_LTE_UUID == msg_type){
        memset(&forbid_lte_uuid,0x0,UUID_NUM*sizeof(u_int32_t));
        if(count){
            memcpy(&forbid_lte_uuid,p,count*sizeof(u_int32_t));
        }
    }
    multi_write_unlock();
    return 0;
}


static int __multi_channel_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh)
{
    int ret = 0;

    switch(nlh->nlmsg_type)
    {
        case SET_MULTI_CHANNEL_ENABLE:
            ret = set_multi_channel_enable(nlh);
            break;

        case SET_MULTI_CHANNEL_IF_UP:
            ret = set_multi_channel_ifup(nlh);
            break;

        case SET_MULTI_CHANNEL_IF_DOWN:
            ret = set_multi_channel_ifdown(nlh);
            break;

        case SET_MULTI_CHANNEL_LTE_MODE:
            ret = set_multi_channel_lte_mode(nlh);
            break;

        case SET_MULTI_CHANNEL_WIFI_MODE:
            ret = set_multi_channel_wifi_mode(nlh);
            break;

        case SET_MULTI_CHANNEL_WIFI_AUTH:
            ret = set_multi_channel_wifi_auth(nlh);
            break;
        case SET_MULTI_CHANNEL_WIFI_UUID:
        case SET_MULTI_CHANNEL_LTE_UUID:
            set_multi_channel_uuid(nlh,nlh->nlmsg_type);
            break;
        default:
            break;
    }

    return ret;

}

static void multi_channel_netlink_rcv(struct sk_buff *skb)
{
    netlink_rcv_skb(skb, &__multi_channel_rcv_msg);
}


static int multi_channel_netlink_init(void)
{
    struct netlink_kernel_cfg cfg = {
        .input = multi_channel_netlink_rcv
    };

    multi_channel_sock = netlink_kernel_create(&init_net, NETLINK_MULTI_CHANNEL,&cfg);

    if (NULL == multi_channel_sock) {
        printk("Create multi_channel_sock sock failed\n");
        return -1;
    }
    return 0;
}

static void  multi_channel_netlink_exit(void)
{
    if (multi_channel_sock) {
        netlink_kernel_release(multi_channel_sock);
    }
    return;
}

static void init_if_info(void)
{
    int i = 0;
    memset(if_info,0x0,IF_NUM*sizeof(struct inface_information));
    for(i = 0; i < IF_NUM; i++){
        if_info[i].idle_rtt = IDLE_FLAG;
    }
    return;
}

static void __exit multi_channel_fini(void)
{
    multi_calc_rtt = NULL;
    del_timer(&multi_channel_timer);
    multi_channel_netlink_exit();
    nf_unregister_hooks(multi_channel_ops, ARRAY_SIZE(multi_channel_ops));
}

static int __init multi_channel_init(void)
{
    memset(&forbid_wifi_uuid,0x0,UUID_NUM*sizeof(u_int32_t));
    memset(&forbid_lte_uuid,0x0,UUID_NUM*sizeof(u_int32_t));
    init_if_info();
    multi_channel_timer_start();
    nf_register_hooks(multi_channel_ops, ARRAY_SIZE(multi_channel_ops));
    multi_channel_netlink_init();
    multi_calc_rtt = calc_tcp_syn_rtt;
    return 0;
}

module_init(multi_channel_init);
module_exit(multi_channel_fini);
