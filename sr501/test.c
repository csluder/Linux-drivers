#include <stdlib.h>
#include <stdio.h>
#include <asm/types.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include <string.h>

main()
{
	int fd;
           struct sockaddr_nl sa;
	   int len;
           char buf[4096];
           struct iovec iov = { buf, sizeof(buf) };
           struct nlmsghdr *nh;
           struct msghdr msg = { &sa, sizeof(sa), &iov, 1, NULL, 0, 0 };
	char *data;

           memset(&sa, 0, sizeof(sa));
           sa.nl_family = AF_NETLINK;
           sa.nl_groups = 1;

           fd = socket(AF_NETLINK, SOCK_RAW, 31);
           bind(fd, (struct sockaddr *) &sa, sizeof(sa));

while (1 == 1){
           len = recvmsg(fd, &msg, 0);


	data = NLMSG_DATA(buf);
	printf("Sensor = %d %s\n", data[0], &data[1]);
}
}
