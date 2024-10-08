/****************************************************************************
 * include/netdb.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NETDB_H
#define __INCLUDE_NETDB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/* Inclusion of the <netdb.h> header may also make visible all symbols from
 * <netinet/in.h>, <sys/socket.h>, and <inttypes.h>.
 */

#include <inttypes.h>

#include <netinet/in.h>
#include <sys/socket.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The <netdb.h> header shall define the IPPORT_RESERVED macro with the
 * value of the highest reserved Internet port number.
 */

#define IPPORT_RESERVED 0xffff /* No reserved port numbers */

/* The <netdb.h> header shall define the following macros that evaluate to
 * bitwise-distinct integer constants for use in the flags field of the
 * addrinfo structure:
 *
 *   AI_PASSIVE      - Socket address is intended for bind().
 *   AI_CANONNAME    - Request for canonical name.
 *   AI_NUMERICHOST  - Return numeric host address as name.
 *   AI_NUMERICSERV  - Inhibit service name resolution.
 *   AI_V4MAPPED     - If no IPv6 addresses are found, query for IPv4
 *                     addresses and return them to the caller as IPv4-mapped
 *                     IPv6 addresses.
 *   AI_ALL          - Query for both IPv4 and IPv6 addresses.
 *   AI_ADDRCONFIG   - Query for IPv4 addresses only when an IPv4 address is
 *                     configured; query for IPv6 addresses only when an IPv6
 *                     address is configured.
 */

#define AI_PASSIVE      (1 << 0)
#define AI_CANONNAME    (1 << 1)
#define AI_NUMERICHOST  (1 << 2)
#define AI_NUMERICSERV  (1 << 3)
#define AI_V4MAPPED     (1 << 4)
#define AI_ALL          (1 << 5)
#define AI_ADDRCONFIG   (1 << 6)

/* The <netdb.h> header shall define the following macros that evaluate to
 * bitwise-distinct integer constants for use in the flags argument to
 * getnameinfo():
 *
 *   NI_NOFQDN       - Only the nodename portion of the FQDN is returned for
 *                     local hosts.
 *   NI_NUMERICHOST  - The numeric form of the node's address is returned
 *                     instead of its name.
 *   NI_NAMEREQD     - Return an error if the node's name cannot be located
 *                     in the database.
 *   NI_NUMERICSERV  - The numeric form of the service address is returned
 *                     instead of its name.
 *   NI_NUMERICSCOPE - For IPv6 addresses, the numeric form of the scope
 *                     identifier is returned instead of its name.
 *   NI_DGRAM        - Indicates that the service is a datagram service
 *                     (SOCK_DGRAM).
 */

#define NI_NOFQDN       (1 << 0)
#define NI_NUMERICHOST  (1 << 1)
#define NI_NAMEREQD     (1 << 2)
#define NI_NUMERICSERV  (1 << 3)
#define NI_NUMERICSCOPE (1 << 4)
#define NI_DGRAM        (1 << 5)

/* Address Information Errors.  The <netdb.h> header shall define the
 * following macros for use as error values for getaddrinfo() and
 * getnameinfo():
 *
 *   EAI_AGAIN       - The name could not be resolved at this time. Future
 *                     attempts may succeed.
 *   EAI_BADFLAGS    - The flags had an invalid value.
 *   EAI_FAIL        - A non-recoverable error occurred.
 *   EAI_FAMILY      - The address family was not recognized or the address
 *                     length was invalid for the specified family.
 *   EAI_MEMORY      - There was a memory allocation failure.
 *   EAI_NONAME      - The name does not resolve for the supplied
 *                     parameters.  NI_NAMEREQD is set and the host's name
 *                     cannot be located, or both nodename and servname were
 *                     null.
 *   EAI_SERVICE     - The service passed was not recognized for the
 *                     specified socket type.
 *   EAI_SOCKTYPE    - The intended socket type was not recognized.
 *   EAI_SYSTEM      - A system error occurred. The error code can be found
 *                     in errno.
 *   EAI_OVERFLOW    - An argument buffer overflowed.
 */

#define EAI_AGAIN       1
#define EAI_BADFLAGS    2
#define EAI_FAIL        3
#define EAI_FAMILY      4
#define EAI_MEMORY      5
#define EAI_NONAME      6
#define EAI_SERVICE     7
#define EAI_SOCKTYPE    8
#define EAI_SYSTEM      9
#define EAI_OVERFLOW    10

/* h_errno values that may be returned by gethosbyname(), gethostbyname_r(),
 * gethostbyaddr(), or gethostbyaddr_r()
 *
 *   HOST_NOT_FOUND - No such host is known.
 *
 *   NO_DATA - The server recognized the request and the name, but no
 *     address is available. Another type of request to the name server
 *     for the domain might return an answer.
 *
 *   NO_RECOVERY - An unexpected server failure occurred which cannot be
 *     recovered.
 *
 *   TRY_AGAIN - A temporary and possibly transient error occurred, such as
 *     a failure of a server to respond.
 *
 * These are obsolete and were removed in the Open Group Base Specifications
 * Issue 7, 2018 edition.
 */

#define HOST_NOT_FOUND 1
#define NO_DATA        2
#define NO_ADDRESS     NO_DATA
#define NO_RECOVERY    3
#define TRY_AGAIN      4

/* NI_MAXHOST is the max of
 *
 *    CONFIG_NETDB_DNSCLIENT_NAMESIZE + 1
 *    INET6_ADDRSTRLEN
 *    INET_ADDRSTRLEN
 *
 * Note: INETxxx_ADDRSTRLEN already includes the terminating NUL.
 * Note: INET6_ADDRSTRLEN > INET_ADDRSTRLEN is assumed.
 */

#if defined(CONFIG_NET_IPv6)
#define _INET_ADDRSTRLEN INET6_ADDRSTRLEN
#else
#define _INET_ADDRSTRLEN INET_ADDRSTRLEN
#endif

#if defined(CONFIG_NETDB_DNSCLIENT) && \
    (CONFIG_NETDB_DNSCLIENT_NAMESIZE + 1) > _INET_ADDRSTRLEN
#define NI_MAXHOST (CONFIG_NETDB_DNSCLIENT_NAMESIZE + 1)
#else
#define NI_MAXHOST _INET_ADDRSTRLEN
#endif

/* Right now, g_services_db[] only has "ntp".
 * 16 should be large enough.
 */

#define NI_MAXSERV 16

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hostent
{
  FAR char  *h_name;       /* Official name of the host. */
  FAR char **h_aliases;    /* A pointer to an array of pointers to the
                            * alternative host names, terminated by a
                            * null pointer. */
  int        h_addrtype;   /* Address type. */
  int        h_length;     /* The length, in bytes, of the address. */
  FAR char **h_addr_list;  /* A pointer to an array of pointers to network
                            * addresses (in network byte order) for the host,
                            * terminated by a null pointer. */
};

#define h_addr h_addr_list[0] /* For backward compatibility */

struct netent
{
  FAR char  *n_name;       /* Official, fully-qualified(including the domain)
                            * name of the host. */
  FAR char **n_aliases;    /* A pointer to an array of pointers to the
                            * alternative network names, terminated by a
                            * null pointer. */
  int        n_addrtype;   /* The address type of the network. */
  uint32_t   n_net;        /* The network number, in host byte order. */
};

struct protoent
{
  FAR char  *p_name;       /* Official name of the protocol. */
  FAR char **p_aliases;    /* A pointer to an array of pointers to
                            * alternative protocol names, terminated by a
                            * null pointer. */
  uint8_t    p_proto;      /* The protocol number. */
  uint8_t    idx;          /* Index used by the local database, required
                            * by _r version APIs */
};

struct servent
{
  FAR char  *s_name;       /* Official name of the service. */
  FAR char **s_aliases;    /* A pointer to an array of pointers to
                            * alternative service names, terminated by a
                            * null pointer. */
  int        s_port;       /* The port number at which the service resides,
                            * in network byte order. */
  FAR char  *s_proto;      /* The name of the protocol to use when
                            * contacting the service. */
};

struct addrinfo
{
  int        ai_flags;     /* Input flags. */
  int        ai_family;    /* Address family of socket. */
  int        ai_socktype;  /* Socket type. */
  int        ai_protocol;  /* Protocol of socket. */
  socklen_t  ai_addrlen;   /* Length of socket address. */

  FAR struct sockaddr *ai_addr;      /* Socket address of socket. */
  FAR char            *ai_canonname; /* Canonical name of service location. */
  FAR struct addrinfo *ai_next;      /* Pointer to next in list. */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* When the <netdb.h> header is included, h_errno shall be available as a
 * modifiable lvalue of type int. It is unspecified whether h_errno is a
 * macro or an identifier declared with external linkage.
 *
 * h_errno is obsolete and was removed in the Open Group Base Specifications
 * Issue 7, 2018 edition.
 */

/* REVISIT:  This should at least be per-task? */

EXTERN int h_errno;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int rexec(FAR char **ahost, int inport, FAR const char *user,
          FAR const char *passwd, FAR const char *cmd, FAR int *fd2p);
int rexec_af(FAR char **ahost, int inport, FAR const char *user,
             FAR const char *passwd, FAR const char *cmd, FAR int *fd2p,
             sa_family_t af);

#ifdef CONFIG_LIBC_NETDB
#if 0 /* None of these are yet supported */

void                 endhostent(void);
void                 endnetent(void);
void                 endprotoent(void);
void                 endservent(void);
#endif
void                 freeaddrinfo(FAR struct addrinfo *ai);
FAR const char      *gai_strerror(int);
int                  getaddrinfo(FAR const char *nodename,
                                 FAR const char *servname,
                                 FAR const struct addrinfo *hints,
                                 FAR struct addrinfo **res);
int                  getnameinfo(FAR const struct sockaddr *sa,
                                 socklen_t salen, FAR char *node,
                                 socklen_t nodelen, FAR char *service,
                                 socklen_t servicelen, int flags);

FAR struct hostent  *gethostbyaddr(FAR const void *addr, socklen_t len,
                                   int type);
FAR struct hostent  *gethostbyname(FAR const char *name);
FAR struct hostent  *gethostbyname2(FAR const char *name, int type);
FAR struct servent  *getservbyport(int port, FAR const char *proto);
FAR struct servent  *getservbyname(FAR const char *name,
                                   FAR const char *proto);

void                 setprotoent(int stayopen);
void                 endprotoent(void);
FAR struct protoent *getprotobyname(FAR const char *name);
FAR struct protoent *getprotobynumber(int proto);
FAR struct protoent *getprotoent(void);

#if 0 /* None of these are yet supported */
FAR struct hostent  *gethostent(void);
FAR struct netent   *getnetbyaddr(uint32_t net, int type);
FAR struct netent   *getnetbyname(FAR const char *name);
FAR struct netent   *getnetent(void);
FAR struct servent  *getservent(void);
void                 sethostent(int);
void                 setnetent(int stayopen);
void                 setservent(int);
#endif /* None of these are yet supported */

/* Non-standard interfaces similar to Glibc 2 interfaces */

int gethostbyaddr_r(FAR const void *addr, socklen_t len, int type,
                    FAR struct hostent *host, FAR char *buf,
                    size_t buflen, FAR struct hostent **result,
                    FAR int *h_errnop);
int gethostbyname_r(FAR const char *name,
                    FAR struct hostent *host, FAR char *buf,
                    size_t buflen, FAR struct hostent **result,
                    FAR int *h_errnop);
int gethostbyname2_r(FAR const char *name, int type,
                     FAR struct hostent *host, FAR char *buf,
                     size_t buflen, FAR struct hostent **result,
                     FAR int *h_errnop);
int getservbyport_r(int port, FAR const char *proto,
                    FAR struct servent *result_buf, FAR char *buf,
                    size_t buflen, FAR struct servent **result);
int getservbyname_r(FAR const char *name, FAR const char *proto,
                    FAR struct servent *result_buf, FAR char *buf,
                    size_t buflen, FAR struct servent **result);

void setprotoent_r(int stayopen, FAR struct protoent *result_buf);
void endprotoent_r(FAR struct protoent *result_buf);
int getprotoent_r(FAR struct protoent *result_buf, FAR char *buf,
                  size_t buflen, FAR struct protoent **result);
int getprotobyname_r(FAR const char *name,
                     FAR struct protoent *result_buf, FAR char *buf,
                     size_t buflen, FAR struct protoent **result);
int getprotobynumber_r(int proto,
                       FAR struct protoent *result_buf, FAR char *buf,
                       size_t buflen, FAR struct protoent **result);

#endif /* CONFIG_LIBC_NETDB */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NETDB_H */
