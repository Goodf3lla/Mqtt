#include <base/component.h>
#include <util/xml_node.h>
#include <os/config.h>
#include <acc_overtaking/acc_overtaking.h>
#include <nic/packet_allocator.h>
#include <lwip/genode.h>
extern "C" {
#include <lwip/sockets.h>
#include <lwip/api.h>
}

Genode::size_t Component::stack_size() { return 64*1024; }

void Component::construct(Genode::Env &)
{
	enum { BUF_SIZE = Nic::Packet_allocator::DEFAULT_PACKET_SIZE * 128 };

	Genode::Xml_node network = Genode::config()->xml_node().sub_node("network");

	if (network.attribute_value<bool>("dhcp", true)) {
		if (lwip_nic_init(0,
						  0,
						  0,
						  BUF_SIZE,
						  BUF_SIZE)) {
			return;
		}
	} else {
		char ip_addr[16] = { 0 };
		char subnet[16] = { 0 };
		char gateway[16] = { 0 };

		try {
			network.attribute("ip-address").value(ip_addr, sizeof(ip_addr));
			network.attribute("subnet").value(subnet, sizeof(subnet));
			network.attribute("gateway").value(gateway, sizeof(gateway));
		} catch(Genode::Xml_node::Nonexistent_attribute) {
			Genode::error("please check the network configuration");
			return;
		}
	}

	acc_overtaking acc_overtaking("acc_overtaking");
}
