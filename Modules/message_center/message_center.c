#include "message_center.h"
#include "stdlib.h"
#include "string.h"
// #include "bsp_log.h"

//message_center是个链表
//通过自引用next_topic_node将所有发布者连接成链表
//message_center是为了不处理链表头的特殊情况
/* message_center是fake head node,是方便链表编写的技巧,这样就不需要处理链表头的特殊情况 */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL,
    .mutex = NULL,
    .pub_registered_flag = 0
    };

static void MutexesInit()
{
    static uint8_t init_flag = 0;
    if (!init_flag) 
    {
        osMutexDef_t mutex_def = {0};
        message_center.mutex = osMutexCreate(&mutex_def);
        init_flag = 1;
    }
}

static void CheckName(char *name)
{
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
    {
        while (1)
            ; // 进入这里说明话题名超出长度限制
    }
}

//需要让发布者和订阅者的消息长度一致。
static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        while (1)
            ; 
    }
}

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    MutexesInit();
    CheckName(name);
     //直接将message_center的指针给node，message_center会直接跳过,不需要特殊处理
    Publisher_t *node = &message_center;
    
    // 获取虚拟头节点锁（保护话题链表遍历）
    osMutexWait(message_center.mutex, osWaitForever);
    
    while (node->next_topic_node)
    {
        node = node->next_topic_node; // 切换到下一个发布者(话题)结点
        //一个发布者只能有发一个话题，已经注册过的topic名会和要注册的topic进行对比，
        //一样就直接返回node，无需再进行所谓的分配内存等
        if (strcmp(node->topic_name, name) == 0)
        {
            CheckLen(data_len, node->data_len);
            node->pub_registered_flag = 1;
            osMutexRelease(message_center.mutex);
            return node;
        }
    }
    
    // 遍历完发现尚未创建name对应的话题
    // 在链表尾部创建新的话题并初始化
    node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->next_topic_node->pub_registered_flag = 1;
    
    // 创建话题节点的互斥锁
    osMutexDef_t mutex_def = {0};
    node->next_topic_node->mutex = osMutexCreate(&mutex_def);
    
    osMutexRelease(message_center.mutex);
    return node->next_topic_node;
}

//其实原理和bsp_modules_app有异曲同工之妙
//因为你会发现订阅者其实与发布者隶属的订阅者地址都是一样的!!
Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    MutexesInit();
    Publisher_t *pub = PubRegister(name, data_len); // 查找或创建该话题的发布者
    
    // 创建新的订阅者结点,申请内存,注意要memset因为新空间不一定是空的,可能有之前留存的垃圾值
    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    memset(ret, 0, sizeof(Subscriber_t));
    
    // 对新建的Subscriber进行初始化
    ret->data_len = data_len;
    for (size_t i = 0; i < QUEUE_SIZE; ++i)
    {
        // 给消息队列的每一个元素分配空间,queue里保存的实际上是数据指针,这样可以兼容不同的数据长度
        ret->queue[i] = malloc(data_len);
    }
    
    // 创建订阅者节点的互斥锁
    osMutexDef_t mutex_def = {0};
    ret->mutex = osMutexCreate(&mutex_def);
    
    // 如果是第一个订阅者,特殊处理一下,将first_subs指针指向新建的订阅者
    if (pub->first_subs == NULL)
    {
        // 获取话题节点锁保护first_subs指针的修改
        osMutexWait(pub->mutex, osWaitForever);
        pub->first_subs = ret;
        osMutexRelease(pub->mutex);
        return ret;
    }
    
    // 若该话题已经有订阅者, 遍历订阅者链表,直到到达尾部
    // 获取话题节点锁保护订阅者链表操作
    osMutexWait(pub->mutex, osWaitForever);
     //这里first_subs的数据类型为Subscriber_t *;所以直接等于就行
    Subscriber_t *sub = pub->first_subs;
    while (sub->next_subs_queue)
    {
        sub = sub->next_subs_queue;
    }
    sub->next_subs_queue = ret;
    
    osMutexRelease(pub->mutex);
    return ret;
}






//相当于接收发布者扔掉的老数据。
//这里你就会发现其实sub和pub–›first_subs其实就是一个地址相同的一个东西
/* 如果队列为空,会返回0;成功获取数据,返回1; */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    // 获取订阅者锁
    if (osMutexWait(sub->mutex, osWaitForever) != osOK) {
        return 0;
    }
    
    if (sub->temp_size == 0)
    {
        osMutexRelease(sub->mutex);
        return 0;
    }
    
    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx + 1) % QUEUE_SIZE;
        //每接收到一个数据证明我需要接收的数据就少一个
    sub->temp_size--;
    
    osMutexRelease(sub->mutex);
    return 1;
}

/*[0][1][2][3]假设这个队列长为4，一开始信息会被放到[0]里，然后依次类推，放[1]、[2]、[3]，此时队列满
了，最老的数据front_idx=0；又来了一条消息，那么最老的[0]应该被删掉,所以[1]变成了front_idx即最老的数据。
队列里就少了一个数据，此时再往这个队列塞数据，最新的数据back_idx应该塞到[0]里，此时队列又满了，[1]需要被替换掉，
所以最老的数据变成了[2],最新的数据back_idx应该塞到[1]里，以此类推。*/


uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    // 获取话题节点锁
    if (osMutexWait(pub->mutex, osWaitForever) != osOK) {
        return 0;
    }
    //要发给发布者对应的订阅者
    Subscriber_t *iter = pub->first_subs;
    
    // 遍历订阅了当前话题的所有订阅者,依次填入最新消息
    while (iter)
    {
        // 获取订阅者锁
        if (osMutexWait(iter->mutex, osWaitForever) == osOK)
        {
            // 遍历订阅了当前话题的所有订阅者,依次填入最新消息
            if (iter->temp_size == QUEUE_SIZE) 
            {
                 //订阅者订阅数据，原理就是上面说过的fifo队列。
                iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
                iter->temp_size--; // 相当于出队,size-1
            }
            // 将Pub的数据复制到队列的尾部(最新)
            memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
            iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE; // 队列尾部前移
            iter->temp_size++;                                  // 入队,size+1
            
            osMutexRelease(iter->mutex);
        }
        
        iter = iter->next_subs_queue; // 访问下一个订阅者
    }
    
    // 释放话题节点锁
    osMutexRelease(pub->mutex);
    return 1;
}