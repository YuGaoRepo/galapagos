-- (c) Copyright 1995-2018 Xilinx, Inc. All rights reserved.
-- 
-- This file contains confidential and proprietary information
-- of Xilinx, Inc. and is protected under U.S. and
-- international copyright and other intellectual property
-- laws.
-- 
-- DISCLAIMER
-- This disclaimer is not a license and does not grant any
-- rights to the materials distributed herewith. Except as
-- otherwise provided in a valid license issued to you by
-- Xilinx, and to the maximum extent permitted by applicable
-- law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
-- WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
-- AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
-- BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
-- INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
-- (2) Xilinx shall not be liable (whether in contract or tort,
-- including negligence, or under any other theory of
-- liability) for any loss or damage of any kind or nature
-- related to, arising under or in connection with these
-- materials, including for any direct, or any indirect,
-- special, incidental, or consequential loss or damage
-- (including loss of data, profits, goodwill, or any type of
-- loss or damage suffered as a result of any action brought
-- by a third party) even if such damage or loss was
-- reasonably foreseeable or Xilinx had been advised of the
-- possibility of the same.
-- 
-- CRITICAL APPLICATIONS
-- Xilinx products are not designed or intended to be fail-
-- safe, or for use in any application requiring fail-safe
-- performance, such as life-support or safety devices or
-- systems, Class III medical devices, nuclear facilities,
-- applications related to the deployment of airbags, or any
-- other applications that could lead to death, personal
-- injury, or severe property or environmental damage
-- (individually and collectively, "Critical
-- Applications"). Customer assumes the sole risk and
-- liability of any use of Xilinx products in Critical
-- Applications, subject only to applicable laws and
-- regulations governing limitations on product liability.
-- 
-- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
-- PART OF THIS FILE AT ALL TIMES.
-- 
-- DO NOT MODIFY THIS FILE.

-- IP VLNV: xilinx.com:ip:axi_datamover:5.1
-- IP Revision: 17

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY axi_datamover_v5_1_17;
USE axi_datamover_v5_1_17.axi_datamover;

ENTITY axi_datamover_0 IS
  PORT (
    m_axi_mm2s_aclk : IN STD_LOGIC;
    m_axi_mm2s_aresetn : IN STD_LOGIC;
    mm2s_err : OUT STD_LOGIC;
    m_axis_mm2s_cmdsts_aclk : IN STD_LOGIC;
    m_axis_mm2s_cmdsts_aresetn : IN STD_LOGIC;
    s_axis_mm2s_cmd_tvalid : IN STD_LOGIC;
    s_axis_mm2s_cmd_tready : OUT STD_LOGIC;
    s_axis_mm2s_cmd_tdata : IN STD_LOGIC_VECTOR(71 DOWNTO 0);
    m_axis_mm2s_sts_tvalid : OUT STD_LOGIC;
    m_axis_mm2s_sts_tready : IN STD_LOGIC;
    m_axis_mm2s_sts_tdata : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    m_axis_mm2s_sts_tkeep : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    m_axis_mm2s_sts_tlast : OUT STD_LOGIC;
    m_axi_mm2s_arid : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    m_axi_mm2s_araddr : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    m_axi_mm2s_arlen : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    m_axi_mm2s_arsize : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
    m_axi_mm2s_arburst : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
    m_axi_mm2s_arprot : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
    m_axi_mm2s_arcache : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    m_axi_mm2s_aruser : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    m_axi_mm2s_arvalid : OUT STD_LOGIC;
    m_axi_mm2s_arready : IN STD_LOGIC;
    m_axi_mm2s_rdata : IN STD_LOGIC_VECTOR(511 DOWNTO 0);
    m_axi_mm2s_rresp : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
    m_axi_mm2s_rlast : IN STD_LOGIC;
    m_axi_mm2s_rvalid : IN STD_LOGIC;
    m_axi_mm2s_rready : OUT STD_LOGIC;
    m_axis_mm2s_tdata : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
    m_axis_mm2s_tkeep : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    m_axis_mm2s_tlast : OUT STD_LOGIC;
    m_axis_mm2s_tvalid : OUT STD_LOGIC;
    m_axis_mm2s_tready : IN STD_LOGIC;
    m_axi_s2mm_aclk : IN STD_LOGIC;
    m_axi_s2mm_aresetn : IN STD_LOGIC;
    s2mm_err : OUT STD_LOGIC;
    m_axis_s2mm_cmdsts_awclk : IN STD_LOGIC;
    m_axis_s2mm_cmdsts_aresetn : IN STD_LOGIC;
    s_axis_s2mm_cmd_tvalid : IN STD_LOGIC;
    s_axis_s2mm_cmd_tready : OUT STD_LOGIC;
    s_axis_s2mm_cmd_tdata : IN STD_LOGIC_VECTOR(71 DOWNTO 0);
    m_axis_s2mm_sts_tvalid : OUT STD_LOGIC;
    m_axis_s2mm_sts_tready : IN STD_LOGIC;
    m_axis_s2mm_sts_tdata : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    m_axis_s2mm_sts_tkeep : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    m_axis_s2mm_sts_tlast : OUT STD_LOGIC;
    m_axi_s2mm_awid : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    m_axi_s2mm_awaddr : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    m_axi_s2mm_awlen : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    m_axi_s2mm_awsize : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
    m_axi_s2mm_awburst : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
    m_axi_s2mm_awprot : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
    m_axi_s2mm_awcache : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    m_axi_s2mm_awuser : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    m_axi_s2mm_awvalid : OUT STD_LOGIC;
    m_axi_s2mm_awready : IN STD_LOGIC;
    m_axi_s2mm_wdata : OUT STD_LOGIC_VECTOR(511 DOWNTO 0);
    m_axi_s2mm_wstrb : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
    m_axi_s2mm_wlast : OUT STD_LOGIC;
    m_axi_s2mm_wvalid : OUT STD_LOGIC;
    m_axi_s2mm_wready : IN STD_LOGIC;
    m_axi_s2mm_bresp : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
    m_axi_s2mm_bvalid : IN STD_LOGIC;
    m_axi_s2mm_bready : OUT STD_LOGIC;
    s_axis_s2mm_tdata : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
    s_axis_s2mm_tkeep : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    s_axis_s2mm_tlast : IN STD_LOGIC;
    s_axis_s2mm_tvalid : IN STD_LOGIC;
    s_axis_s2mm_tready : OUT STD_LOGIC
  );
END axi_datamover_0;

ARCHITECTURE axi_datamover_0_arch OF axi_datamover_0 IS
  ATTRIBUTE DowngradeIPIdentifiedWarnings : STRING;
  ATTRIBUTE DowngradeIPIdentifiedWarnings OF axi_datamover_0_arch: ARCHITECTURE IS "yes";
  COMPONENT axi_datamover IS
    GENERIC (
      C_INCLUDE_MM2S : INTEGER;
      C_M_AXI_MM2S_ARID : INTEGER;
      C_M_AXI_MM2S_ID_WIDTH : INTEGER;
      C_M_AXI_MM2S_ADDR_WIDTH : INTEGER;
      C_M_AXI_MM2S_DATA_WIDTH : INTEGER;
      C_M_AXIS_MM2S_TDATA_WIDTH : INTEGER;
      C_INCLUDE_MM2S_STSFIFO : INTEGER;
      C_MM2S_STSCMD_FIFO_DEPTH : INTEGER;
      C_MM2S_STSCMD_IS_ASYNC : INTEGER;
      C_INCLUDE_MM2S_DRE : INTEGER;
      C_MM2S_BURST_SIZE : INTEGER;
      C_MM2S_BTT_USED : INTEGER;
      C_MM2S_ADDR_PIPE_DEPTH : INTEGER;
      C_INCLUDE_S2MM : INTEGER;
      C_M_AXI_S2MM_AWID : INTEGER;
      C_M_AXI_S2MM_ID_WIDTH : INTEGER;
      C_M_AXI_S2MM_ADDR_WIDTH : INTEGER;
      C_M_AXI_S2MM_DATA_WIDTH : INTEGER;
      C_S_AXIS_S2MM_TDATA_WIDTH : INTEGER;
      C_INCLUDE_S2MM_STSFIFO : INTEGER;
      C_S2MM_STSCMD_FIFO_DEPTH : INTEGER;
      C_S2MM_STSCMD_IS_ASYNC : INTEGER;
      C_INCLUDE_S2MM_DRE : INTEGER;
      C_S2MM_BURST_SIZE : INTEGER;
      C_S2MM_BTT_USED : INTEGER;
      C_S2MM_SUPPORT_INDET_BTT : INTEGER;
      C_S2MM_ADDR_PIPE_DEPTH : INTEGER;
      C_FAMILY : STRING;
      C_MM2S_INCLUDE_SF : INTEGER;
      C_S2MM_INCLUDE_SF : INTEGER;
      C_ENABLE_CACHE_USER : INTEGER;
      C_ENABLE_MM2S_TKEEP : INTEGER;
      C_ENABLE_S2MM_TKEEP : INTEGER;
      C_ENABLE_SKID_BUF : STRING;
      C_ENABLE_S2MM_ADV_SIG : INTEGER;
      C_ENABLE_MM2S_ADV_SIG : INTEGER;
      C_CMD_WIDTH : INTEGER
    );
    PORT (
      m_axi_mm2s_aclk : IN STD_LOGIC;
      m_axi_mm2s_aresetn : IN STD_LOGIC;
      mm2s_halt : IN STD_LOGIC;
      mm2s_halt_cmplt : OUT STD_LOGIC;
      mm2s_err : OUT STD_LOGIC;
      m_axis_mm2s_cmdsts_aclk : IN STD_LOGIC;
      m_axis_mm2s_cmdsts_aresetn : IN STD_LOGIC;
      s_axis_mm2s_cmd_tvalid : IN STD_LOGIC;
      s_axis_mm2s_cmd_tready : OUT STD_LOGIC;
      s_axis_mm2s_cmd_tdata : IN STD_LOGIC_VECTOR(71 DOWNTO 0);
      m_axis_mm2s_sts_tvalid : OUT STD_LOGIC;
      m_axis_mm2s_sts_tready : IN STD_LOGIC;
      m_axis_mm2s_sts_tdata : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      m_axis_mm2s_sts_tkeep : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
      m_axis_mm2s_sts_tlast : OUT STD_LOGIC;
      mm2s_allow_addr_req : IN STD_LOGIC;
      mm2s_addr_req_posted : OUT STD_LOGIC;
      mm2s_rd_xfer_cmplt : OUT STD_LOGIC;
      m_axi_mm2s_arid : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_mm2s_araddr : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
      m_axi_mm2s_arlen : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      m_axi_mm2s_arsize : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
      m_axi_mm2s_arburst : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
      m_axi_mm2s_arprot : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
      m_axi_mm2s_arcache : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_mm2s_aruser : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_mm2s_arvalid : OUT STD_LOGIC;
      m_axi_mm2s_arready : IN STD_LOGIC;
      m_axi_mm2s_rdata : IN STD_LOGIC_VECTOR(511 DOWNTO 0);
      m_axi_mm2s_rresp : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
      m_axi_mm2s_rlast : IN STD_LOGIC;
      m_axi_mm2s_rvalid : IN STD_LOGIC;
      m_axi_mm2s_rready : OUT STD_LOGIC;
      m_axis_mm2s_tdata : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
      m_axis_mm2s_tkeep : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      m_axis_mm2s_tlast : OUT STD_LOGIC;
      m_axis_mm2s_tvalid : OUT STD_LOGIC;
      m_axis_mm2s_tready : IN STD_LOGIC;
      mm2s_dbg_sel : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
      mm2s_dbg_data : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
      m_axi_s2mm_aclk : IN STD_LOGIC;
      m_axi_s2mm_aresetn : IN STD_LOGIC;
      s2mm_halt : IN STD_LOGIC;
      s2mm_halt_cmplt : OUT STD_LOGIC;
      s2mm_err : OUT STD_LOGIC;
      m_axis_s2mm_cmdsts_awclk : IN STD_LOGIC;
      m_axis_s2mm_cmdsts_aresetn : IN STD_LOGIC;
      s_axis_s2mm_cmd_tvalid : IN STD_LOGIC;
      s_axis_s2mm_cmd_tready : OUT STD_LOGIC;
      s_axis_s2mm_cmd_tdata : IN STD_LOGIC_VECTOR(71 DOWNTO 0);
      m_axis_s2mm_sts_tvalid : OUT STD_LOGIC;
      m_axis_s2mm_sts_tready : IN STD_LOGIC;
      m_axis_s2mm_sts_tdata : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      m_axis_s2mm_sts_tkeep : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
      m_axis_s2mm_sts_tlast : OUT STD_LOGIC;
      s2mm_allow_addr_req : IN STD_LOGIC;
      s2mm_addr_req_posted : OUT STD_LOGIC;
      s2mm_wr_xfer_cmplt : OUT STD_LOGIC;
      s2mm_ld_nxt_len : OUT STD_LOGIC;
      s2mm_wr_len : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      m_axi_s2mm_awid : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_s2mm_awaddr : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
      m_axi_s2mm_awlen : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      m_axi_s2mm_awsize : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
      m_axi_s2mm_awburst : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
      m_axi_s2mm_awprot : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
      m_axi_s2mm_awcache : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_s2mm_awuser : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_s2mm_awvalid : OUT STD_LOGIC;
      m_axi_s2mm_awready : IN STD_LOGIC;
      m_axi_s2mm_wdata : OUT STD_LOGIC_VECTOR(511 DOWNTO 0);
      m_axi_s2mm_wstrb : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
      m_axi_s2mm_wlast : OUT STD_LOGIC;
      m_axi_s2mm_wvalid : OUT STD_LOGIC;
      m_axi_s2mm_wready : IN STD_LOGIC;
      m_axi_s2mm_bresp : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
      m_axi_s2mm_bvalid : IN STD_LOGIC;
      m_axi_s2mm_bready : OUT STD_LOGIC;
      s_axis_s2mm_tdata : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
      s_axis_s2mm_tkeep : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
      s_axis_s2mm_tlast : IN STD_LOGIC;
      s_axis_s2mm_tvalid : IN STD_LOGIC;
      s_axis_s2mm_tready : OUT STD_LOGIC;
      s2mm_dbg_sel : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
      s2mm_dbg_data : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
    );
  END COMPONENT axi_datamover;
  ATTRIBUTE X_CORE_INFO : STRING;
  ATTRIBUTE X_CORE_INFO OF axi_datamover_0_arch: ARCHITECTURE IS "axi_datamover,Vivado 2017.4";
  ATTRIBUTE CHECK_LICENSE_TYPE : STRING;
  ATTRIBUTE CHECK_LICENSE_TYPE OF axi_datamover_0_arch : ARCHITECTURE IS "axi_datamover_0,axi_datamover,{}";
  ATTRIBUTE CORE_GENERATION_INFO : STRING;
  ATTRIBUTE CORE_GENERATION_INFO OF axi_datamover_0_arch: ARCHITECTURE IS "axi_datamover_0,axi_datamover,{x_ipProduct=Vivado 2017.4,x_ipVendor=xilinx.com,x_ipLibrary=ip,x_ipName=axi_datamover,x_ipVersion=5.1,x_ipCoreRevision=17,x_ipLanguage=VERILOG,x_ipSimLanguage=MIXED,C_INCLUDE_MM2S=1,C_M_AXI_MM2S_ARID=0,C_M_AXI_MM2S_ID_WIDTH=4,C_M_AXI_MM2S_ADDR_WIDTH=32,C_M_AXI_MM2S_DATA_WIDTH=512,C_M_AXIS_MM2S_TDATA_WIDTH=64,C_INCLUDE_MM2S_STSFIFO=1,C_MM2S_STSCMD_FIFO_DEPTH=4,C_MM2S_STSCMD_IS_ASYNC=0,C_INCLUDE_MM2S_DRE=1,C_MM2S_BURST_SIZE=16,C_MM2S_BTT_USED=16,C_MM2S_ADDR_PIPE_DEPT" & 
"H=3,C_INCLUDE_S2MM=1,C_M_AXI_S2MM_AWID=0,C_M_AXI_S2MM_ID_WIDTH=4,C_M_AXI_S2MM_ADDR_WIDTH=32,C_M_AXI_S2MM_DATA_WIDTH=512,C_S_AXIS_S2MM_TDATA_WIDTH=64,C_INCLUDE_S2MM_STSFIFO=1,C_S2MM_STSCMD_FIFO_DEPTH=4,C_S2MM_STSCMD_IS_ASYNC=0,C_INCLUDE_S2MM_DRE=1,C_S2MM_BURST_SIZE=16,C_S2MM_BTT_USED=16,C_S2MM_SUPPORT_INDET_BTT=0,C_S2MM_ADDR_PIPE_DEPTH=4,C_FAMILY=virtex7,C_MM2S_INCLUDE_SF=1,C_S2MM_INCLUDE_SF=1,C_ENABLE_CACHE_USER=0,C_ENABLE_MM2S_TKEEP=1,C_ENABLE_S2MM_TKEEP=1,C_ENABLE_SKID_BUF=11111,C_ENABLE_S2MM_" & 
"ADV_SIG=0,C_ENABLE_MM2S_ADV_SIG=0,C_CMD_WIDTH=72}";
  ATTRIBUTE X_INTERFACE_INFO : STRING;
  ATTRIBUTE X_INTERFACE_PARAMETER : STRING;
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_s2mm_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_S2MM TREADY";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_s2mm_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_S2MM TVALID";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_s2mm_tlast: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_S2MM TLAST";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_s2mm_tkeep: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_S2MM TKEEP";
  ATTRIBUTE X_INTERFACE_PARAMETER OF s_axis_s2mm_tdata: SIGNAL IS "XIL_INTERFACENAME S_AXIS_S2MM, TDATA_NUM_BYTES 8, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.000, LAYERED_METADATA undef";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_s2mm_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_S2MM TDATA";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_bready: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM BREADY";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_bvalid: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM BVALID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_bresp: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM BRESP";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_wready: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WREADY";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_wvalid: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WVALID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_wlast: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WLAST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_wstrb: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WSTRB";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_wdata: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WDATA";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awready: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWREADY";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awvalid: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWVALID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awuser: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWUSER";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awcache: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWCACHE";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awprot: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWPROT";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awburst: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWBURST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awsize: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWSIZE";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awlen: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWLEN";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awaddr: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWADDR";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axi_s2mm_awid: SIGNAL IS "XIL_INTERFACENAME M_AXI_S2MM, NUM_WRITE_OUTSTANDING 2, DATA_WIDTH 512, PROTOCOL AXI4, FREQ_HZ 100000000, ID_WIDTH 4, ADDR_WIDTH 32, AWUSER_WIDTH 4, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE WRITE_ONLY, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 0, SUPPORTS_NARROW_BURST 1, NUM_READ_OUTSTANDING 2, MAX_BURST_LENGTH 256, PHASE 0.000, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_awid: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_s2mm_sts_tlast: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_S2MM_STS TLAST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_s2mm_sts_tkeep: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_S2MM_STS TKEEP";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_s2mm_sts_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_S2MM_STS TDATA";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_s2mm_sts_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_S2MM_STS TREADY";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axis_s2mm_sts_tvalid: SIGNAL IS "XIL_INTERFACENAME M_AXIS_S2MM_STS, TDATA_NUM_BYTES 1, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.000, LAYERED_METADATA undef";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_s2mm_sts_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_S2MM_STS TVALID";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_s2mm_cmd_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_S2MM_CMD TDATA";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_s2mm_cmd_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_S2MM_CMD TREADY";
  ATTRIBUTE X_INTERFACE_PARAMETER OF s_axis_s2mm_cmd_tvalid: SIGNAL IS "XIL_INTERFACENAME S_AXIS_S2MM_CMD, TDATA_NUM_BYTES 9, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 0, HAS_TLAST 0, FREQ_HZ 100000000, PHASE 0.000, LAYERED_METADATA undef";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_s2mm_cmd_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_S2MM_CMD TVALID";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axis_s2mm_cmdsts_aresetn: SIGNAL IS "XIL_INTERFACENAME M_AXIS_S2MM_CMDSTS_ARESETN, POLARITY ACTIVE_LOW";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_s2mm_cmdsts_aresetn: SIGNAL IS "xilinx.com:signal:reset:1.0 M_AXIS_S2MM_CMDSTS_ARESETN RST";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axis_s2mm_cmdsts_awclk: SIGNAL IS "XIL_INTERFACENAME M_AXIS_S2MM_CMDSTS_AWCLK, ASSOCIATED_BUSIF S_AXIS_S2MM_CMD:M_AXIS_S2MM_STS, ASSOCIATED_RESET m_axis_s2mm_cmdsts_aresetn, FREQ_HZ 100000000, PHASE 0.000";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_s2mm_cmdsts_awclk: SIGNAL IS "xilinx.com:signal:clock:1.0 M_AXIS_S2MM_CMDSTS_AWCLK CLK";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axi_s2mm_aresetn: SIGNAL IS "XIL_INTERFACENAME M_AXI_S2MM_ARESETN, POLARITY ACTIVE_LOW";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_aresetn: SIGNAL IS "xilinx.com:signal:reset:1.0 M_AXI_S2MM_ARESETN RST";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axi_s2mm_aclk: SIGNAL IS "XIL_INTERFACENAME M_AXI_S2MM_ACLK, ASSOCIATED_BUSIF M_AXI_S2MM:S_AXIS_S2MM, ASSOCIATED_RESET m_axi_s2mm_aresetn, FREQ_HZ 100000000, PHASE 0.000";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_s2mm_aclk: SIGNAL IS "xilinx.com:signal:clock:1.0 M_AXI_S2MM_ACLK CLK";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TREADY";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TVALID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_tlast: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TLAST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_tkeep: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TKEEP";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axis_mm2s_tdata: SIGNAL IS "XIL_INTERFACENAME M_AXIS_MM2S, TDATA_NUM_BYTES 8, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.000, LAYERED_METADATA undef";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TDATA";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_rready: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RREADY";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_rvalid: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RVALID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_rlast: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RLAST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_rresp: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RRESP";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_rdata: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RDATA";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_arready: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARREADY";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_arvalid: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARVALID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_aruser: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARUSER";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_arcache: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARCACHE";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_arprot: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARPROT";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_arburst: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARBURST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_arsize: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARSIZE";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_arlen: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARLEN";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_araddr: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARADDR";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axi_mm2s_arid: SIGNAL IS "XIL_INTERFACENAME M_AXI_MM2S, NUM_READ_OUTSTANDING 2, DATA_WIDTH 512, PROTOCOL AXI4, FREQ_HZ 100000000, ID_WIDTH 4, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 4, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_ONLY, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 0, HAS_BRESP 0, HAS_RRESP 1, SUPPORTS_NARROW_BURST 1, NUM_WRITE_OUTSTANDING 2, MAX_BURST_LENGTH 256, PHASE 0.000, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_arid: SIGNAL IS "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_sts_tlast: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S_STS TLAST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_sts_tkeep: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S_STS TKEEP";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_sts_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S_STS TDATA";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_sts_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S_STS TREADY";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axis_mm2s_sts_tvalid: SIGNAL IS "XIL_INTERFACENAME M_AXIS_MM2S_STS, TDATA_NUM_BYTES 1, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.000, LAYERED_METADATA undef";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_sts_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_MM2S_STS TVALID";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_mm2s_cmd_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_MM2S_CMD TDATA";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_mm2s_cmd_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_MM2S_CMD TREADY";
  ATTRIBUTE X_INTERFACE_PARAMETER OF s_axis_mm2s_cmd_tvalid: SIGNAL IS "XIL_INTERFACENAME S_AXIS_MM2S_CMD, TDATA_NUM_BYTES 9, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 0, HAS_TLAST 0, FREQ_HZ 100000000, PHASE 0.000, LAYERED_METADATA undef";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_mm2s_cmd_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_MM2S_CMD TVALID";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axis_mm2s_cmdsts_aresetn: SIGNAL IS "XIL_INTERFACENAME M_AXIS_MM2S_CMDSTS_ARESETN, POLARITY ACTIVE_LOW";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_cmdsts_aresetn: SIGNAL IS "xilinx.com:signal:reset:1.0 M_AXIS_MM2S_CMDSTS_ARESETN RST";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axis_mm2s_cmdsts_aclk: SIGNAL IS "XIL_INTERFACENAME M_AXIS_MM2S_CMDSTS_ACLK, ASSOCIATED_BUSIF S_AXIS_MM2S_CMD:M_AXIS_MM2S_STS, ASSOCIATED_RESET m_axis_mm2s_cmdsts_aresetn, FREQ_HZ 100000000, PHASE 0.000";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_mm2s_cmdsts_aclk: SIGNAL IS "xilinx.com:signal:clock:1.0 M_AXIS_MM2S_CMDSTS_ACLK CLK";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axi_mm2s_aresetn: SIGNAL IS "XIL_INTERFACENAME M_AXI_MM2S_ARESETN, POLARITY ACTIVE_LOW";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_aresetn: SIGNAL IS "xilinx.com:signal:reset:1.0 M_AXI_MM2S_ARESETN RST";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axi_mm2s_aclk: SIGNAL IS "XIL_INTERFACENAME M_AXI_MM2S_ACLK, ASSOCIATED_BUSIF M_AXI_MM2S:M_AXIS_MM2S, ASSOCIATED_RESET m_axi_mm2s_aresetn, FREQ_HZ 100000000, PHASE 0.000";
  ATTRIBUTE X_INTERFACE_INFO OF m_axi_mm2s_aclk: SIGNAL IS "xilinx.com:signal:clock:1.0 M_AXI_MM2S_ACLK CLK";
BEGIN
  U0 : axi_datamover
    GENERIC MAP (
      C_INCLUDE_MM2S => 1,
      C_M_AXI_MM2S_ARID => 0,
      C_M_AXI_MM2S_ID_WIDTH => 4,
      C_M_AXI_MM2S_ADDR_WIDTH => 32,
      C_M_AXI_MM2S_DATA_WIDTH => 512,
      C_M_AXIS_MM2S_TDATA_WIDTH => 64,
      C_INCLUDE_MM2S_STSFIFO => 1,
      C_MM2S_STSCMD_FIFO_DEPTH => 4,
      C_MM2S_STSCMD_IS_ASYNC => 0,
      C_INCLUDE_MM2S_DRE => 1,
      C_MM2S_BURST_SIZE => 16,
      C_MM2S_BTT_USED => 16,
      C_MM2S_ADDR_PIPE_DEPTH => 3,
      C_INCLUDE_S2MM => 1,
      C_M_AXI_S2MM_AWID => 0,
      C_M_AXI_S2MM_ID_WIDTH => 4,
      C_M_AXI_S2MM_ADDR_WIDTH => 32,
      C_M_AXI_S2MM_DATA_WIDTH => 512,
      C_S_AXIS_S2MM_TDATA_WIDTH => 64,
      C_INCLUDE_S2MM_STSFIFO => 1,
      C_S2MM_STSCMD_FIFO_DEPTH => 4,
      C_S2MM_STSCMD_IS_ASYNC => 0,
      C_INCLUDE_S2MM_DRE => 1,
      C_S2MM_BURST_SIZE => 16,
      C_S2MM_BTT_USED => 16,
      C_S2MM_SUPPORT_INDET_BTT => 0,
      C_S2MM_ADDR_PIPE_DEPTH => 4,
      C_FAMILY => "virtex7",
      C_MM2S_INCLUDE_SF => 1,
      C_S2MM_INCLUDE_SF => 1,
      C_ENABLE_CACHE_USER => 0,
      C_ENABLE_MM2S_TKEEP => 1,
      C_ENABLE_S2MM_TKEEP => 1,
      C_ENABLE_SKID_BUF => "11111",
      C_ENABLE_S2MM_ADV_SIG => 0,
      C_ENABLE_MM2S_ADV_SIG => 0,
      C_CMD_WIDTH => 72
    )
    PORT MAP (
      m_axi_mm2s_aclk => m_axi_mm2s_aclk,
      m_axi_mm2s_aresetn => m_axi_mm2s_aresetn,
      mm2s_halt => '0',
      mm2s_err => mm2s_err,
      m_axis_mm2s_cmdsts_aclk => m_axis_mm2s_cmdsts_aclk,
      m_axis_mm2s_cmdsts_aresetn => m_axis_mm2s_cmdsts_aresetn,
      s_axis_mm2s_cmd_tvalid => s_axis_mm2s_cmd_tvalid,
      s_axis_mm2s_cmd_tready => s_axis_mm2s_cmd_tready,
      s_axis_mm2s_cmd_tdata => s_axis_mm2s_cmd_tdata,
      m_axis_mm2s_sts_tvalid => m_axis_mm2s_sts_tvalid,
      m_axis_mm2s_sts_tready => m_axis_mm2s_sts_tready,
      m_axis_mm2s_sts_tdata => m_axis_mm2s_sts_tdata,
      m_axis_mm2s_sts_tkeep => m_axis_mm2s_sts_tkeep,
      m_axis_mm2s_sts_tlast => m_axis_mm2s_sts_tlast,
      mm2s_allow_addr_req => '1',
      m_axi_mm2s_arid => m_axi_mm2s_arid,
      m_axi_mm2s_araddr => m_axi_mm2s_araddr,
      m_axi_mm2s_arlen => m_axi_mm2s_arlen,
      m_axi_mm2s_arsize => m_axi_mm2s_arsize,
      m_axi_mm2s_arburst => m_axi_mm2s_arburst,
      m_axi_mm2s_arprot => m_axi_mm2s_arprot,
      m_axi_mm2s_arcache => m_axi_mm2s_arcache,
      m_axi_mm2s_aruser => m_axi_mm2s_aruser,
      m_axi_mm2s_arvalid => m_axi_mm2s_arvalid,
      m_axi_mm2s_arready => m_axi_mm2s_arready,
      m_axi_mm2s_rdata => m_axi_mm2s_rdata,
      m_axi_mm2s_rresp => m_axi_mm2s_rresp,
      m_axi_mm2s_rlast => m_axi_mm2s_rlast,
      m_axi_mm2s_rvalid => m_axi_mm2s_rvalid,
      m_axi_mm2s_rready => m_axi_mm2s_rready,
      m_axis_mm2s_tdata => m_axis_mm2s_tdata,
      m_axis_mm2s_tkeep => m_axis_mm2s_tkeep,
      m_axis_mm2s_tlast => m_axis_mm2s_tlast,
      m_axis_mm2s_tvalid => m_axis_mm2s_tvalid,
      m_axis_mm2s_tready => m_axis_mm2s_tready,
      mm2s_dbg_sel => STD_LOGIC_VECTOR(TO_UNSIGNED(0, 4)),
      m_axi_s2mm_aclk => m_axi_s2mm_aclk,
      m_axi_s2mm_aresetn => m_axi_s2mm_aresetn,
      s2mm_halt => '0',
      s2mm_err => s2mm_err,
      m_axis_s2mm_cmdsts_awclk => m_axis_s2mm_cmdsts_awclk,
      m_axis_s2mm_cmdsts_aresetn => m_axis_s2mm_cmdsts_aresetn,
      s_axis_s2mm_cmd_tvalid => s_axis_s2mm_cmd_tvalid,
      s_axis_s2mm_cmd_tready => s_axis_s2mm_cmd_tready,
      s_axis_s2mm_cmd_tdata => s_axis_s2mm_cmd_tdata,
      m_axis_s2mm_sts_tvalid => m_axis_s2mm_sts_tvalid,
      m_axis_s2mm_sts_tready => m_axis_s2mm_sts_tready,
      m_axis_s2mm_sts_tdata => m_axis_s2mm_sts_tdata,
      m_axis_s2mm_sts_tkeep => m_axis_s2mm_sts_tkeep,
      m_axis_s2mm_sts_tlast => m_axis_s2mm_sts_tlast,
      s2mm_allow_addr_req => '1',
      m_axi_s2mm_awid => m_axi_s2mm_awid,
      m_axi_s2mm_awaddr => m_axi_s2mm_awaddr,
      m_axi_s2mm_awlen => m_axi_s2mm_awlen,
      m_axi_s2mm_awsize => m_axi_s2mm_awsize,
      m_axi_s2mm_awburst => m_axi_s2mm_awburst,
      m_axi_s2mm_awprot => m_axi_s2mm_awprot,
      m_axi_s2mm_awcache => m_axi_s2mm_awcache,
      m_axi_s2mm_awuser => m_axi_s2mm_awuser,
      m_axi_s2mm_awvalid => m_axi_s2mm_awvalid,
      m_axi_s2mm_awready => m_axi_s2mm_awready,
      m_axi_s2mm_wdata => m_axi_s2mm_wdata,
      m_axi_s2mm_wstrb => m_axi_s2mm_wstrb,
      m_axi_s2mm_wlast => m_axi_s2mm_wlast,
      m_axi_s2mm_wvalid => m_axi_s2mm_wvalid,
      m_axi_s2mm_wready => m_axi_s2mm_wready,
      m_axi_s2mm_bresp => m_axi_s2mm_bresp,
      m_axi_s2mm_bvalid => m_axi_s2mm_bvalid,
      m_axi_s2mm_bready => m_axi_s2mm_bready,
      s_axis_s2mm_tdata => s_axis_s2mm_tdata,
      s_axis_s2mm_tkeep => s_axis_s2mm_tkeep,
      s_axis_s2mm_tlast => s_axis_s2mm_tlast,
      s_axis_s2mm_tvalid => s_axis_s2mm_tvalid,
      s_axis_s2mm_tready => s_axis_s2mm_tready,
      s2mm_dbg_sel => STD_LOGIC_VECTOR(TO_UNSIGNED(0, 4))
    );
END axi_datamover_0_arch;
