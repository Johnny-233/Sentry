"""
CRC 查找表生成与验证

- CRC-8/MAXIM（反射 / LSB-first）：裁判系统帧头校验
  多项式 0x31（反射 0x8C），初值 0xFF
- CRC-16（裁判系统官方实现，反射 / LSB-first）：裁判系统 + 图传整包校验
  多项式 0x1021（反射 0x8408），初值 0xFFFF
  注意：这是反射实现，check("123456789") = 0x6F91，与教科书上
  非反射的 CRC-16/CCITT-FALSE（check = 0x29B1）不是同一种。

用法:
    python crc8_gen.py          # 生成并打印 C 数组 + 自检
"""

# 已知标准 check 值（CRC 目录 / 官方定义的校验向量）
CRC8_MAXIM_CHECK = 0xA1         # CRC-8/MAXIM("123456789", init=0x00)
CRC16_REF_CHECK = 0x6F91        # 反射 CCITT("123456789", init=0xFFFF, 无末尾 xorout)


def crc8_reflected_table():
    """生成 CRC-8/MAXIM 反射表（poly 0x8C，LSB-first）"""
    poly_rev = 0x8C
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ poly_rev if (crc & 0x01) else (crc >> 1)
        table.append(crc & 0xFF)
    return table


def crc8_bitwise(data, init=0xFF):
    """位运算版 CRC-8/MAXIM（反射），用于交叉验证表"""
    poly_rev = 0x8C
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ poly_rev if (crc & 0x01) else (crc >> 1)
    return crc & 0xFF


def crc16_reflected_table():
    """生成裁判系统 CRC16 反射表（poly 0x8408，配合 (crc>>8)^tab[(crc^data)&0xFF]）"""
    poly_rev = 0x8408  # 0x1021 的反转
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ poly_rev if (crc & 0x01) else (crc >> 1)
        table.append(crc & 0xFFFF)
    return table


def crc16_bitwise(data, init=0xFFFF):
    """位运算版 CRC16（反射），用于交叉验证表"""
    poly_rev = 0x8408
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ poly_rev if (crc & 0x01) else (crc >> 1)
    return crc & 0xFFFF


def crc16_table_driven(data, table, init=0xFFFF):
    """表驱动版 CRC16，与 crc.c 的 CRC16_Calculate 一致"""
    crc = init
    for b in data:
        crc = (crc >> 8) ^ table[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def print_c_table(name, table, bits):
    fmt = '0x{:02X}' if bits == 8 else '0x{:04X}'
    typ = 'uint8_t' if bits == 8 else 'uint16_t'
    print(f"static const {typ} {name}[256] = {{")
    for i in range(0, 256, 8):
        row = ', '.join(fmt.format(table[j]) for j in range(i, min(i + 8, 256)))
        print(f"    {row},")
    print("};")


def main():
    # ---- CRC-8/MAXIM（反射）----
    crc8_tab = crc8_reflected_table()

    # 独立验证 1：标准 check 值（init=0x00）
    assert crc8_bitwise(b"123456789", init=0x00) == CRC8_MAXIM_CHECK, \
        "CRC-8/MAXIM check 值不匹配"
    # 独立验证 2：表与位运算逐字节一致（init=0x00 作为建表基准）
    for i in range(256):
        assert crc8_tab[i] == crc8_bitwise(bytes([i]), init=0x00), \
            f"CRC8 table[{i}] 与位运算不一致"

    print("CRC8（CRC-8/MAXIM，反射 LSB-first，poly 0x8C，init 0xFF）")
    print("裁判系统帧头校验")
    print_c_table("crc8_tab", crc8_tab, 8)

    # ---- CRC16（反射实现）----
    crc16_tab = crc16_reflected_table()

    # 独立验证 1：标准 check 值
    assert crc16_bitwise(b"123456789") == CRC16_REF_CHECK, \
        "CRC16 反射 check 值不匹配"
    # 独立验证 2：表驱动 == 位运算（多个测试串）
    assert crc16_tab[:4] == [0x0000, 0x1189, 0x2312, 0x329B], "CRC16 表头不匹配"
    for s in (b"123456789", b"", bytes(range(1, 256)), b"ABC", b"\x00\xff"):
        assert crc16_table_driven(s, crc16_tab) == crc16_bitwise(s), \
            f"CRC16 表驱动与位运算不一致: {s!r}"

    print("\n\nCRC16（裁判系统官方实现，反射 LSB-first，poly 0x8408，init 0xFFFF）")
    print("裁判系统 + 图传整包校验")
    print_c_table("crc16_tab", crc16_tab, 16)
    print(f"\n样例：'123456789' 的 CRC16 = 0x{crc16_bitwise(b'123456789'):04X} "
          f"(期望 0x{CRC16_REF_CHECK:04X})")

    print("\n[OK] 全部自检通过")


if __name__ == '__main__':
    main()
