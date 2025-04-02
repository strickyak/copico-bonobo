# this is about fixing the coco3 boot problems.
# This data is the input from a coco3 trace,
# modified and simplified, but gime_init_gen
# will reduce this further.

nums = [
        (0xff90, 0x6c),
        (0xff91, 0x00),
        (0xff92, 0x00),
        (0xff93, 0x00),
        (0xff94, 0x09),
        (0xff95, 0x00),
        (0xff96, 0x00),
        (0xff97, 0x00),
        (0xff98, 0x03),
        (0xff99, 0x20),
        (0xff9a, 0x00),
        (0xff9b, 0x00),
        (0xff9c, 0x00),
        (0xff9d, 0x3c),
        (0xff9e, 0x01),
        (0xff9f, 0x00),
        (0xff90, 0x80),
        (0xff90, 0x0a),
        (0xffde, 0x00),
        (0xffb0, 0x12),
        (0xffb1, 0x12),
        (0xffb2, 0x12),
        (0xffb3, 0x12),
        (0xffb4, 0x12),
        (0xffb5, 0x12),
        (0xffb6, 0x12),
        (0xffb7, 0x12),
        (0xffb8, 0x12),
        (0xffb9, 0x12),
        (0xffba, 0x12),
        (0xffbb, 0x12),
        (0xffbc, 0x12),
        (0xffbd, 0x12),
        (0xffbe, 0x12),
        (0xffbf, 0x12),
        (0xffa0, 0x38),
        (0xffa1, 0x39),
        (0xffa2, 0x34),
        (0xffa3, 0x3b),
        (0xffa4, 0x3c),
        (0xffa5, 0x3d),
        (0xffa6, 0x3e),
        (0xffa7, 0x3f),
        (0xffa8, 0x38),
        (0xffa9, 0x30),
        (0xffaa, 0x31),
        (0xffab, 0x32),
        (0xffac, 0x33),
        (0xffad, 0x3d),
        (0xffae, 0x35),
        (0xffaf, 0x3f),
        (0xff90, 0xce),
        (0xff94, 0xff),
        (0xff95, 0xff),
        (0xff98, 0x00),
        (0xff99, 0x00),
        (0xff9a, 0x00),
        (0xff9b, 0x00),
        (0xff9c, 0x0f),
        (0xff9d, 0xe0),
        (0xff9e, 0x00),
        (0xff9f, 0x00),
        (0xff21, 0x00),
        (0xff23, 0x00),
        (0xff20, 0xfe),
        (0xff22, 0xf8),
        (0xff21, 0x34),
        (0xff23, 0x34),
        (0xff22, 0x00),
        (0xff20, 0x02),
        (0xff01, 0x00),
        (0xff03, 0x00),
        (0xff00, 0x00),
        (0xff02, 0xff),
        (0xff01, 0x34),
        (0xff03, 0x34),
        (0xffc0, 0xff),
        (0xffc2, 0xff),
        (0xffc4, 0xff),
        (0xffc6, 0xff),
        (0xffc8, 0xff),
        (0xffca, 0xff),
        (0xffcc, 0xff),
        (0xffce, 0xff),
        (0xffd0, 0xff),
        (0xffd2, 0xff),
        (0xffd4, 0xff),
        (0xffd6, 0xff),
        (0xffc9, 0xff),
        (0xff02, 0x00),
        (0xffd5, 0xff),
        (0xff02, 0xdf),
        (0xff02, 0xef),
        (0xff90, 0xca),
        (0xffde, 0x00),
        (0xffdf, 0x00),
        #
        (0xffde, 0x00),
        (0xffdf, 0x00),
        (0xffdf, 0x00),
        (0xff90, 0xce),
        (0xffb0, 0x12),
        (0xffb1, 0x24),
        (0xffb2, 0x0b),
        (0xffb3, 0x07),
        (0xffb4, 0x3f),
        (0xffb5, 0x1f),
        (0xffb6, 0x09),
        (0xffb7, 0x26),
        (0xffb8, 0x00),
        (0xffb9, 0x12),
        (0xffba, 0x00),
        (0xffbb, 0x3f),
        (0xffbc, 0x00),
        (0xffbd, 0x12),
        (0xffbe, 0x00),
        (0xffbf, 0x26),
        (0xffa2, 0x3a),
        (0xff21, 0x00),
        (0xff23, 0x00),
        (0xff20, 0xfe),
        (0xff22, 0xf8),
        (0xff21, 0x34),
        (0xff23, 0x34),
        (0xff22, 0x00),
        (0xff20, 0x02),
        (0xff01, 0x00),
        (0xff03, 0x00),
        (0xff00, 0x00),
        (0xff02, 0xff),
        (0xff01, 0x34),
        (0xff03, 0x34),
        (0xff23, 0x37),
        (0xffa0, 0x34),
        (0xffa0, 0x38),
        (0xff03, 0x81),
        (0xffd2, 0x45),
        (0xffd0, 0x45),
        (0xffce, 0x45),
        (0xffcc, 0x45),
        (0xffca, 0x45),
        (0xffc9, 0x45),
        (0xffc6, 0x45),
        (0xffc4, 0x45),
        (0xffc2, 0x45),
        (0xffc0, 0x45),
        (0xff22, 0x07),
        (0xffd2, 0x58),
        (0xffd0, 0x58),
        (0xffce, 0x58),
        (0xffcc, 0x58),
        (0xffca, 0x58),
        (0xffc9, 0x58),
        (0xffc6, 0x58),
        (0xffc4, 0x58),
        (0xffc2, 0x58),
        (0xffc0, 0x58),
        (0xff22, 0x07),
        (0xffd2, 0x54),
        (0xffd0, 0x54),
        (0xffce, 0x54),
        (0xffcc, 0x54),
        (0xffca, 0x54),
        (0xffc9, 0x54),
        (0xffc6, 0x54),
        (0xffc4, 0x54),
        (0xffc2, 0x54),
        (0xffc0, 0x54),
        (0xff22, 0x07),
        (0xffd2, 0x45),
        (0xffd0, 0x45),
        (0xffce, 0x45),
        (0xffcc, 0x45),
        (0xffca, 0x45),
        (0xffc9, 0x45),
        (0xffc6, 0x45),
        (0xffc4, 0x45),
        (0xffc2, 0x45),
        (0xffc0, 0x45),
        (0xff22, 0x07),
        (0xffd2, 0x4e),
        (0xffd0, 0x4e),
        (0xffce, 0x4e),
        (0xffcc, 0x4e),
        (0xffca, 0x4e),
        (0xffc9, 0x4e),
        (0xffc6, 0x4e),
        (0xffc4, 0x4e),
        (0xffc2, 0x4e),
        (0xffc0, 0x4e),
        (0xff22, 0x07),
        (0xffd2, 0x44),
        (0xffd0, 0x44),
        (0xffce, 0x44),
        (0xffcc, 0x44),
        (0xffca, 0x44),
        (0xffc9, 0x44),
        (0xffc6, 0x44),
        (0xffc4, 0x44),
        (0xffc2, 0x44),
        (0xffc0, 0x44),
        (0xff22, 0x07),
        (0xffd2, 0x45),
        (0xffd0, 0x45),
        (0xffce, 0x45),
        (0xffcc, 0x45),
        (0xffca, 0x45),
        (0xffc9, 0x45),
        (0xffc6, 0x45),
        (0xffc4, 0x45),
        (0xffc2, 0x45),
        (0xffc0, 0x45),
        (0xff22, 0x07),
        (0xffd2, 0x44),
        (0xffd0, 0x44),
        (0xffce, 0x44),
        (0xffcc, 0x44),
        (0xffca, 0x44),
        (0xffc9, 0x44),
        (0xffc6, 0x44),
        (0xffc4, 0x44),
        (0xffc2, 0x44),
        (0xffc0, 0x44),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x43),
        (0xffd0, 0x43),
        (0xffce, 0x43),
        (0xffcc, 0x43),
        (0xffca, 0x43),
        (0xffc9, 0x43),
        (0xffc6, 0x43),
        (0xffc4, 0x43),
        (0xffc2, 0x43),
        (0xffc0, 0x43),
        (0xff22, 0x07),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x4c),
        (0xffd0, 0x4c),
        (0xffce, 0x4c),
        (0xffcc, 0x4c),
        (0xffca, 0x4c),
        (0xffc9, 0x4c),
        (0xffc6, 0x4c),
        (0xffc4, 0x4c),
        (0xffc2, 0x4c),
        (0xffc0, 0x4c),
        (0xff22, 0x07),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x52),
        (0xffd0, 0x52),
        (0xffce, 0x52),
        (0xffcc, 0x52),
        (0xffca, 0x52),
        (0xffc9, 0x52),
        (0xffc6, 0x52),
        (0xffc4, 0x52),
        (0xffc2, 0x52),
        (0xffc0, 0x52),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x42),
        (0xffd0, 0x42),
        (0xffce, 0x42),
        (0xffcc, 0x42),
        (0xffca, 0x42),
        (0xffc9, 0x42),
        (0xffc6, 0x42),
        (0xffc4, 0x42),
        (0xffc2, 0x42),
        (0xffc0, 0x42),
        (0xff22, 0x07),
        (0xffd2, 0x41),
        (0xffd0, 0x41),
        (0xffce, 0x41),
        (0xffcc, 0x41),
        (0xffca, 0x41),
        (0xffc9, 0x41),
        (0xffc6, 0x41),
        (0xffc4, 0x41),
        (0xffc2, 0x41),
        (0xffc0, 0x41),
        (0xff22, 0x07),
        (0xffd2, 0x53),
        (0xffd0, 0x53),
        (0xffce, 0x53),
        (0xffcc, 0x53),
        (0xffca, 0x53),
        (0xffc9, 0x53),
        (0xffc6, 0x53),
        (0xffc4, 0x53),
        (0xffc2, 0x53),
        (0xffc0, 0x53),
        (0xff22, 0x07),
        (0xffd2, 0x49),
        (0xffd0, 0x49),
        (0xffce, 0x49),
        (0xffcc, 0x49),
        (0xffca, 0x49),
        (0xffc9, 0x49),
        (0xffc6, 0x49),
        (0xffc4, 0x49),
        (0xffc2, 0x49),
        (0xffc0, 0x49),
        (0xff22, 0x07),
        (0xffd2, 0x43),
        (0xffd0, 0x43),
        (0xffce, 0x43),
        (0xffcc, 0x43),
        (0xffca, 0x43),
        (0xffc9, 0x43),
        (0xffc6, 0x43),
        (0xffc4, 0x43),
        (0xffc2, 0x43),
        (0xffc0, 0x43),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x32),
        (0xffd0, 0x32),
        (0xffce, 0x32),
        (0xffcc, 0x32),
        (0xffca, 0x32),
        (0xffc9, 0x32),
        (0xffc6, 0x32),
        (0xffc4, 0x32),
        (0xffc2, 0x32),
        (0xffc0, 0x32),
        (0xff22, 0x07),
        (0xffd2, 0x2e),
        (0xffd0, 0x2e),
        (0xffce, 0x2e),
        (0xffcc, 0x2e),
        (0xffca, 0x2e),
        (0xffc9, 0x2e),
        (0xffc6, 0x2e),
        (0xffc4, 0x2e),
        (0xffc2, 0x2e),
        (0xffc0, 0x2e),
        (0xff22, 0x07),
        (0xffd2, 0x30),
        (0xffd0, 0x30),
        (0xffce, 0x30),
        (0xffcc, 0x30),
        (0xffca, 0x30),
        (0xffc9, 0x30),
        (0xffc6, 0x30),
        (0xffc4, 0x30),
        (0xffc2, 0x30),
        (0xffc0, 0x30),
        (0xff22, 0x07),
        (0xffd2, 0x0d),
        (0xffd0, 0x0d),
        (0xffce, 0x0d),
        (0xffcc, 0x0d),
        (0xffca, 0x0d),
        (0xffc9, 0x0d),
        (0xffc6, 0x0d),
        (0xffc4, 0x0d),
        (0xffc2, 0x0d),
        (0xffc0, 0x0d),
        (0xff22, 0x07),
        (0xffd2, 0x43),
        (0xffd0, 0x43),
        (0xffce, 0x43),
        (0xffcc, 0x43),
        (0xffca, 0x43),
        (0xffc9, 0x43),
        (0xffc6, 0x43),
        (0xffc4, 0x43),
        (0xffc2, 0x43),
        (0xffc0, 0x43),
        (0xff22, 0x07),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x50),
        (0xffd0, 0x50),
        (0xffce, 0x50),
        (0xffcc, 0x50),
        (0xffca, 0x50),
        (0xffc9, 0x50),
        (0xffc6, 0x50),
        (0xffc4, 0x50),
        (0xffc2, 0x50),
        (0xffc0, 0x50),
        (0xff22, 0x07),
        (0xffd2, 0x52),
        (0xffd0, 0x52),
        (0xffce, 0x52),
        (0xffcc, 0x52),
        (0xffca, 0x52),
        (0xffc9, 0x52),
        (0xffc6, 0x52),
        (0xffc4, 0x52),
        (0xffc2, 0x52),
        (0xffc0, 0x52),
        (0xff22, 0x07),
        (0xffd2, 0x2e),
        (0xffd0, 0x2e),
        (0xffce, 0x2e),
        (0xffcc, 0x2e),
        (0xffca, 0x2e),
        (0xffc9, 0x2e),
        (0xffc6, 0x2e),
        (0xffc4, 0x2e),
        (0xffc2, 0x2e),
        (0xffc0, 0x2e),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x31),
        (0xffd0, 0x31),
        (0xffce, 0x31),
        (0xffcc, 0x31),
        (0xffca, 0x31),
        (0xffc9, 0x31),
        (0xffc6, 0x31),
        (0xffc4, 0x31),
        (0xffc2, 0x31),
        (0xffc0, 0x31),
        (0xff22, 0x07),
        (0xffd2, 0x39),
        (0xffd0, 0x39),
        (0xffce, 0x39),
        (0xffcc, 0x39),
        (0xffca, 0x39),
        (0xffc9, 0x39),
        (0xffc6, 0x39),
        (0xffc4, 0x39),
        (0xffc2, 0x39),
        (0xffc0, 0x39),
        (0xff22, 0x07),
        (0xffd2, 0x38),
        (0xffd0, 0x38),
        (0xffce, 0x38),
        (0xffcc, 0x38),
        (0xffca, 0x38),
        (0xffc9, 0x38),
        (0xffc6, 0x38),
        (0xffc4, 0x38),
        (0xffc2, 0x38),
        (0xffc0, 0x38),
        (0xff22, 0x07),
        (0xffd2, 0x32),
        (0xffd0, 0x32),
        (0xffce, 0x32),
        (0xffcc, 0x32),
        (0xffca, 0x32),
        (0xffc9, 0x32),
        (0xffc6, 0x32),
        (0xffc4, 0x32),
        (0xffc2, 0x32),
        (0xffc0, 0x32),
        (0xff22, 0x07),
        (0xffd2, 0x2c),
        (0xffd0, 0x2c),
        (0xffce, 0x2c),
        (0xffcc, 0x2c),
        (0xffca, 0x2c),
        (0xffc9, 0x2c),
        (0xffc6, 0x2c),
        (0xffc4, 0x2c),
        (0xffc2, 0x2c),
        (0xffc0, 0x2c),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x31),
        (0xffd0, 0x31),
        (0xffce, 0x31),
        (0xffcc, 0x31),
        (0xffca, 0x31),
        (0xffc9, 0x31),
        (0xffc6, 0x31),
        (0xffc4, 0x31),
        (0xffc2, 0x31),
        (0xffc0, 0x31),
        (0xff22, 0x07),
        (0xffd2, 0x39),
        (0xffd0, 0x39),
        (0xffce, 0x39),
        (0xffcc, 0x39),
        (0xffca, 0x39),
        (0xffc9, 0x39),
        (0xffc6, 0x39),
        (0xffc4, 0x39),
        (0xffc2, 0x39),
        (0xffc0, 0x39),
        (0xff22, 0x07),
        (0xffd2, 0x38),
        (0xffd0, 0x38),
        (0xffce, 0x38),
        (0xffcc, 0x38),
        (0xffca, 0x38),
        (0xffc9, 0x38),
        (0xffc6, 0x38),
        (0xffc4, 0x38),
        (0xffc2, 0x38),
        (0xffc0, 0x38),
        (0xff22, 0x07),
        (0xffd2, 0x36),
        (0xffd0, 0x36),
        (0xffce, 0x36),
        (0xffcc, 0x36),
        (0xffca, 0x36),
        (0xffc9, 0x36),
        (0xffc6, 0x36),
        (0xffc4, 0x36),
        (0xffc2, 0x36),
        (0xffc0, 0x36),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x42),
        (0xffd0, 0x42),
        (0xffce, 0x42),
        (0xffcc, 0x42),
        (0xffca, 0x42),
        (0xffc9, 0x42),
        (0xffc6, 0x42),
        (0xffc4, 0x42),
        (0xffc2, 0x42),
        (0xffc0, 0x42),
        (0xff22, 0x07),
        (0xffd2, 0x59),
        (0xffd0, 0x59),
        (0xffce, 0x59),
        (0xffcc, 0x59),
        (0xffca, 0x59),
        (0xffc9, 0x59),
        (0xffc6, 0x59),
        (0xffc4, 0x59),
        (0xffc2, 0x59),
        (0xffc0, 0x59),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x54),
        (0xffd0, 0x54),
        (0xffce, 0x54),
        (0xffcc, 0x54),
        (0xffca, 0x54),
        (0xffc9, 0x54),
        (0xffc6, 0x54),
        (0xffc4, 0x54),
        (0xffc2, 0x54),
        (0xffc0, 0x54),
        (0xff22, 0x07),
        (0xffd2, 0x41),
        (0xffd0, 0x41),
        (0xffce, 0x41),
        (0xffcc, 0x41),
        (0xffca, 0x41),
        (0xffc9, 0x41),
        (0xffc6, 0x41),
        (0xffc4, 0x41),
        (0xffc2, 0x41),
        (0xffc0, 0x41),
        (0xff22, 0x07),
        (0xffd2, 0x4e),
        (0xffd0, 0x4e),
        (0xffce, 0x4e),
        (0xffcc, 0x4e),
        (0xffca, 0x4e),
        (0xffc9, 0x4e),
        (0xffc6, 0x4e),
        (0xffc4, 0x4e),
        (0xffc2, 0x4e),
        (0xffc0, 0x4e),
        (0xff22, 0x07),
        (0xffd2, 0x44),
        (0xffd0, 0x44),
        (0xffce, 0x44),
        (0xffcc, 0x44),
        (0xffca, 0x44),
        (0xffc9, 0x44),
        (0xffc6, 0x44),
        (0xffc4, 0x44),
        (0xffc2, 0x44),
        (0xffc0, 0x44),
        (0xff22, 0x07),
        (0xffd2, 0x59),
        (0xffd0, 0x59),
        (0xffce, 0x59),
        (0xffcc, 0x59),
        (0xffca, 0x59),
        (0xffc9, 0x59),
        (0xffc6, 0x59),
        (0xffc4, 0x59),
        (0xffc2, 0x59),
        (0xffc0, 0x59),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x0d),
        (0xffd0, 0x0d),
        (0xffce, 0x0d),
        (0xffcc, 0x0d),
        (0xffca, 0x0d),
        (0xffc9, 0x0d),
        (0xffc6, 0x0d),
        (0xffc4, 0x0d),
        (0xffc2, 0x0d),
        (0xffc0, 0x0d),
        (0xff22, 0x07),
        (0xffd2, 0x55),
        (0xffd0, 0x55),
        (0xffce, 0x55),
        (0xffcc, 0x55),
        (0xffca, 0x55),
        (0xffc9, 0x55),
        (0xffc6, 0x55),
        (0xffc4, 0x55),
        (0xffc2, 0x55),
        (0xffc0, 0x55),
        (0xff22, 0x07),
        (0xffd2, 0x4e),
        (0xffd0, 0x4e),
        (0xffce, 0x4e),
        (0xffcc, 0x4e),
        (0xffca, 0x4e),
        (0xffc9, 0x4e),
        (0xffc6, 0x4e),
        (0xffc4, 0x4e),
        (0xffc2, 0x4e),
        (0xffc0, 0x4e),
        (0xff22, 0x07),
        (0xffd2, 0x44),
        (0xffd0, 0x44),
        (0xffce, 0x44),
        (0xffcc, 0x44),
        (0xffca, 0x44),
        (0xffc9, 0x44),
        (0xffc6, 0x44),
        (0xffc4, 0x44),
        (0xffc2, 0x44),
        (0xffc0, 0x44),
        (0xff22, 0x07),
        (0xffd2, 0x45),
        (0xffd0, 0x45),
        (0xffce, 0x45),
        (0xffcc, 0x45),
        (0xffca, 0x45),
        (0xffc9, 0x45),
        (0xffc6, 0x45),
        (0xffc4, 0x45),
        (0xffc2, 0x45),
        (0xffc0, 0x45),
        (0xff22, 0x07),
        (0xffd2, 0x52),
        (0xffd0, 0x52),
        (0xffce, 0x52),
        (0xffcc, 0x52),
        (0xffca, 0x52),
        (0xffc9, 0x52),
        (0xffc6, 0x52),
        (0xffc4, 0x52),
        (0xffc2, 0x52),
        (0xffc0, 0x52),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x4c),
        (0xffd0, 0x4c),
        (0xffce, 0x4c),
        (0xffcc, 0x4c),
        (0xffca, 0x4c),
        (0xffc9, 0x4c),
        (0xffc6, 0x4c),
        (0xffc4, 0x4c),
        (0xffc2, 0x4c),
        (0xffc0, 0x4c),
        (0xff22, 0x07),
        (0xffd2, 0x49),
        (0xffd0, 0x49),
        (0xffce, 0x49),
        (0xffcc, 0x49),
        (0xffca, 0x49),
        (0xffc9, 0x49),
        (0xffc6, 0x49),
        (0xffc4, 0x49),
        (0xffc2, 0x49),
        (0xffc0, 0x49),
        (0xff22, 0x07),
        (0xffd2, 0x43),
        (0xffd0, 0x43),
        (0xffce, 0x43),
        (0xffcc, 0x43),
        (0xffca, 0x43),
        (0xffc9, 0x43),
        (0xffc6, 0x43),
        (0xffc4, 0x43),
        (0xffc2, 0x43),
        (0xffc0, 0x43),
        (0xff22, 0x07),
        (0xffd2, 0x45),
        (0xffd0, 0x45),
        (0xffce, 0x45),
        (0xffcc, 0x45),
        (0xffca, 0x45),
        (0xffc9, 0x45),
        (0xffc6, 0x45),
        (0xffc4, 0x45),
        (0xffc2, 0x45),
        (0xffc0, 0x45),
        (0xff22, 0x07),
        (0xffd2, 0x4e),
        (0xffd0, 0x4e),
        (0xffce, 0x4e),
        (0xffcc, 0x4e),
        (0xffca, 0x4e),
        (0xffc9, 0x4e),
        (0xffc6, 0x4e),
        (0xffc4, 0x4e),
        (0xffc2, 0x4e),
        (0xffc0, 0x4e),
        (0xff22, 0x07),
        (0xffd2, 0x53),
        (0xffd0, 0x53),
        (0xffce, 0x53),
        (0xffcc, 0x53),
        (0xffca, 0x53),
        (0xffc9, 0x53),
        (0xffc6, 0x53),
        (0xffc4, 0x53),
        (0xffc2, 0x53),
        (0xffc0, 0x53),
        (0xff22, 0x07),
        (0xffd2, 0x45),
        (0xffd0, 0x45),
        (0xffce, 0x45),
        (0xffcc, 0x45),
        (0xffca, 0x45),
        (0xffc9, 0x45),
        (0xffc6, 0x45),
        (0xffc4, 0x45),
        (0xffc2, 0x45),
        (0xffc0, 0x45),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x46),
        (0xffd0, 0x46),
        (0xffce, 0x46),
        (0xffcc, 0x46),
        (0xffca, 0x46),
        (0xffc9, 0x46),
        (0xffc6, 0x46),
        (0xffc4, 0x46),
        (0xffc2, 0x46),
        (0xffc0, 0x46),
        (0xff22, 0x07),
        (0xffd2, 0x52),
        (0xffd0, 0x52),
        (0xffce, 0x52),
        (0xffcc, 0x52),
        (0xffca, 0x52),
        (0xffc9, 0x52),
        (0xffc6, 0x52),
        (0xffc4, 0x52),
        (0xffc2, 0x52),
        (0xffc0, 0x52),
        (0xff22, 0x07),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x4d),
        (0xffd0, 0x4d),
        (0xffce, 0x4d),
        (0xffcc, 0x4d),
        (0xffca, 0x4d),
        (0xffc9, 0x4d),
        (0xffc6, 0x4d),
        (0xffc4, 0x4d),
        (0xffc2, 0x4d),
        (0xffc0, 0x4d),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x4d),
        (0xffd0, 0x4d),
        (0xffce, 0x4d),
        (0xffcc, 0x4d),
        (0xffca, 0x4d),
        (0xffc9, 0x4d),
        (0xffc6, 0x4d),
        (0xffc4, 0x4d),
        (0xffc2, 0x4d),
        (0xffc0, 0x4d),
        (0xff22, 0x07),
        (0xffd2, 0x49),
        (0xffd0, 0x49),
        (0xffce, 0x49),
        (0xffcc, 0x49),
        (0xffca, 0x49),
        (0xffc9, 0x49),
        (0xffc6, 0x49),
        (0xffc4, 0x49),
        (0xffc2, 0x49),
        (0xffc0, 0x49),
        (0xff22, 0x07),
        (0xffd2, 0x43),
        (0xffd0, 0x43),
        (0xffce, 0x43),
        (0xffcc, 0x43),
        (0xffca, 0x43),
        (0xffc9, 0x43),
        (0xffc6, 0x43),
        (0xffc4, 0x43),
        (0xffc2, 0x43),
        (0xffc0, 0x43),
        (0xff22, 0x07),
        (0xffd2, 0x52),
        (0xffd0, 0x52),
        (0xffce, 0x52),
        (0xffcc, 0x52),
        (0xffca, 0x52),
        (0xffc9, 0x52),
        (0xffc6, 0x52),
        (0xffc4, 0x52),
        (0xffc2, 0x52),
        (0xffc0, 0x52),
        (0xff22, 0x07),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x53),
        (0xffd0, 0x53),
        (0xffce, 0x53),
        (0xffcc, 0x53),
        (0xffca, 0x53),
        (0xffc9, 0x53),
        (0xffc6, 0x53),
        (0xffc4, 0x53),
        (0xffc2, 0x53),
        (0xffc0, 0x53),
        (0xff22, 0x07),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x46),
        (0xffd0, 0x46),
        (0xffce, 0x46),
        (0xffcc, 0x46),
        (0xffca, 0x46),
        (0xffc9, 0x46),
        (0xffc6, 0x46),
        (0xffc4, 0x46),
        (0xffc2, 0x46),
        (0xffc0, 0x46),
        (0xff22, 0x07),
        (0xffd2, 0x54),
        (0xffd0, 0x54),
        (0xffce, 0x54),
        (0xffcc, 0x54),
        (0xffca, 0x54),
        (0xffc9, 0x54),
        (0xffc6, 0x54),
        (0xffc4, 0x54),
        (0xffc2, 0x54),
        (0xffc0, 0x54),
        (0xff22, 0x07),
        (0xffd2, 0x0d),
        (0xffd0, 0x0d),
        (0xffce, 0x0d),
        (0xffcc, 0x0d),
        (0xffca, 0x0d),
        (0xffc9, 0x0d),
        (0xffc6, 0x0d),
        (0xffc4, 0x0d),
        (0xffc2, 0x0d),
        (0xffc0, 0x0d),
        (0xff22, 0x07),
        (0xffd2, 0x41),
        (0xffd0, 0x41),
        (0xffce, 0x41),
        (0xffcc, 0x41),
        (0xffca, 0x41),
        (0xffc9, 0x41),
        (0xffc6, 0x41),
        (0xffc4, 0x41),
        (0xffc2, 0x41),
        (0xffc0, 0x41),
        (0xff22, 0x07),
        (0xffd2, 0x4e),
        (0xffd0, 0x4e),
        (0xffce, 0x4e),
        (0xffcc, 0x4e),
        (0xffca, 0x4e),
        (0xffc9, 0x4e),
        (0xffc6, 0x4e),
        (0xffc4, 0x4e),
        (0xffc2, 0x4e),
        (0xffc0, 0x4e),
        (0xff22, 0x07),
        (0xffd2, 0x44),
        (0xffd0, 0x44),
        (0xffce, 0x44),
        (0xffcc, 0x44),
        (0xffca, 0x44),
        (0xffc9, 0x44),
        (0xffc6, 0x44),
        (0xffc4, 0x44),
        (0xffc2, 0x44),
        (0xffc0, 0x44),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x4d),
        (0xffd0, 0x4d),
        (0xffce, 0x4d),
        (0xffcc, 0x4d),
        (0xffca, 0x4d),
        (0xffc9, 0x4d),
        (0xffc6, 0x4d),
        (0xffc4, 0x4d),
        (0xffc2, 0x4d),
        (0xffc0, 0x4d),
        (0xff22, 0x07),
        (0xffd2, 0x49),
        (0xffd0, 0x49),
        (0xffce, 0x49),
        (0xffcc, 0x49),
        (0xffca, 0x49),
        (0xffc9, 0x49),
        (0xffc6, 0x49),
        (0xffc4, 0x49),
        (0xffc2, 0x49),
        (0xffc0, 0x49),
        (0xff22, 0x07),
        (0xffd2, 0x43),
        (0xffd0, 0x43),
        (0xffce, 0x43),
        (0xffcc, 0x43),
        (0xffca, 0x43),
        (0xffc9, 0x43),
        (0xffc6, 0x43),
        (0xffc4, 0x43),
        (0xffc2, 0x43),
        (0xffc0, 0x43),
        (0xff22, 0x07),
        (0xffd2, 0x52),
        (0xffd0, 0x52),
        (0xffce, 0x52),
        (0xffcc, 0x52),
        (0xffca, 0x52),
        (0xffc9, 0x52),
        (0xffc6, 0x52),
        (0xffc4, 0x52),
        (0xffc2, 0x52),
        (0xffc0, 0x52),
        (0xff22, 0x07),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x57),
        (0xffd0, 0x57),
        (0xffce, 0x57),
        (0xffcc, 0x57),
        (0xffca, 0x57),
        (0xffc9, 0x57),
        (0xffc6, 0x57),
        (0xffc4, 0x57),
        (0xffc2, 0x57),
        (0xffc0, 0x57),
        (0xff22, 0x07),
        (0xffd2, 0x41),
        (0xffd0, 0x41),
        (0xffce, 0x41),
        (0xffcc, 0x41),
        (0xffca, 0x41),
        (0xffc9, 0x41),
        (0xffc6, 0x41),
        (0xffc4, 0x41),
        (0xffc2, 0x41),
        (0xffc0, 0x41),
        (0xff22, 0x07),
        (0xffd2, 0x52),
        (0xffd0, 0x52),
        (0xffce, 0x52),
        (0xffcc, 0x52),
        (0xffca, 0x52),
        (0xffc9, 0x52),
        (0xffc6, 0x52),
        (0xffc4, 0x52),
        (0xffc2, 0x52),
        (0xffc0, 0x52),
        (0xff22, 0x07),
        (0xffd2, 0x45),
        (0xffd0, 0x45),
        (0xffce, 0x45),
        (0xffcc, 0x45),
        (0xffca, 0x45),
        (0xffc9, 0x45),
        (0xffc6, 0x45),
        (0xffc4, 0x45),
        (0xffc2, 0x45),
        (0xffc0, 0x45),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x53),
        (0xffd0, 0x53),
        (0xffce, 0x53),
        (0xffcc, 0x53),
        (0xffca, 0x53),
        (0xffc9, 0x53),
        (0xffc6, 0x53),
        (0xffc4, 0x53),
        (0xffc2, 0x53),
        (0xffc0, 0x53),
        (0xff22, 0x07),
        (0xffd2, 0x59),
        (0xffd0, 0x59),
        (0xffce, 0x59),
        (0xffcc, 0x59),
        (0xffca, 0x59),
        (0xffc9, 0x59),
        (0xffc6, 0x59),
        (0xffc4, 0x59),
        (0xffc2, 0x59),
        (0xffc0, 0x59),
        (0xff22, 0x07),
        (0xffd2, 0x53),
        (0xffd0, 0x53),
        (0xffce, 0x53),
        (0xffcc, 0x53),
        (0xffca, 0x53),
        (0xffc9, 0x53),
        (0xffc6, 0x53),
        (0xffc4, 0x53),
        (0xffc2, 0x53),
        (0xffc0, 0x53),
        (0xff22, 0x07),
        (0xffd2, 0x54),
        (0xffd0, 0x54),
        (0xffce, 0x54),
        (0xffcc, 0x54),
        (0xffca, 0x54),
        (0xffc9, 0x54),
        (0xffc6, 0x54),
        (0xffc4, 0x54),
        (0xffc2, 0x54),
        (0xffc0, 0x54),
        (0xff22, 0x07),
        (0xffd2, 0x45),
        (0xffd0, 0x45),
        (0xffce, 0x45),
        (0xffcc, 0x45),
        (0xffca, 0x45),
        (0xffc9, 0x45),
        (0xffc6, 0x45),
        (0xffc4, 0x45),
        (0xffc2, 0x45),
        (0xffc0, 0x45),
        (0xff22, 0x07),
        (0xffd2, 0x4d),
        (0xffd0, 0x4d),
        (0xffce, 0x4d),
        (0xffcc, 0x4d),
        (0xffca, 0x4d),
        (0xffc9, 0x4d),
        (0xffc6, 0x4d),
        (0xffc4, 0x4d),
        (0xffc2, 0x4d),
        (0xffc0, 0x4d),
        (0xff22, 0x07),
        (0xffd2, 0x53),
        (0xffd0, 0x53),
        (0xffce, 0x53),
        (0xffcc, 0x53),
        (0xffca, 0x53),
        (0xffc9, 0x53),
        (0xffc6, 0x53),
        (0xffc4, 0x53),
        (0xffc2, 0x53),
        (0xffc0, 0x53),
        (0xff22, 0x07),
        (0xffd2, 0x20),
        (0xffd0, 0x20),
        (0xffce, 0x20),
        (0xffcc, 0x20),
        (0xffca, 0x20),
        (0xffc9, 0x20),
        (0xffc6, 0x20),
        (0xffc4, 0x20),
        (0xffc2, 0x20),
        (0xffc0, 0x20),
        (0xff22, 0x07),
        (0xffd2, 0x43),
        (0xffd0, 0x43),
        (0xffce, 0x43),
        (0xffcc, 0x43),
        (0xffca, 0x43),
        (0xffc9, 0x43),
        (0xffc6, 0x43),
        (0xffc4, 0x43),
        (0xffc2, 0x43),
        (0xffc0, 0x43),
        (0xff22, 0x07),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x52),
        (0xffd0, 0x52),
        (0xffce, 0x52),
        (0xffcc, 0x52),
        (0xffca, 0x52),
        (0xffc9, 0x52),
        (0xffc6, 0x52),
        (0xffc4, 0x52),
        (0xffc2, 0x52),
        (0xffc0, 0x52),
        (0xff22, 0x07),
        (0xffd2, 0x50),
        (0xffd0, 0x50),
        (0xffce, 0x50),
        (0xffcc, 0x50),
        (0xffca, 0x50),
        (0xffc9, 0x50),
        (0xffc6, 0x50),
        (0xffc4, 0x50),
        (0xffc2, 0x50),
        (0xffc0, 0x50),
        (0xff22, 0x07),
        (0xffd2, 0x2e),
        (0xffd0, 0x2e),
        (0xffce, 0x2e),
        (0xffcc, 0x2e),
        (0xffca, 0x2e),
        (0xffc9, 0x2e),
        (0xffc6, 0x2e),
        (0xffc4, 0x2e),
        (0xffc2, 0x2e),
        (0xffc0, 0x2e),
        (0xff22, 0x07),
        (0xffd2, 0x0d),
        (0xffd0, 0x0d),
        (0xffce, 0x0d),
        (0xffcc, 0x0d),
        (0xffca, 0x0d),
        (0xffc9, 0x0d),
        (0xffc6, 0x0d),
        (0xffc4, 0x0d),
        (0xffc2, 0x0d),
        (0xffc0, 0x0d),
        (0xff22, 0x07),
        (0xffd2, 0x0d),
        (0xffd0, 0x0d),
        (0xffce, 0x0d),
        (0xffcc, 0x0d),
        (0xffca, 0x0d),
        (0xffc9, 0x0d),
        (0xffc6, 0x0d),
        (0xffc4, 0x0d),
        (0xffc2, 0x0d),
        (0xffc0, 0x0d),
        (0xff22, 0x07),
        (0xff90, 0xcc),
        (0xff98, 0x00),
        (0xff99, 0x00),
        (0xff9a, 0x00),
        (0xff9b, 0x00),
        (0xff9c, 0x0f),
        (0xff9d, 0xe0),
        (0xff9e, 0x00),
        (0xff9f, 0x00),
        (0xffa0, 0x34),
        (0xffa0, 0x38),
        (0xffd2, 0x4f),
        (0xffd0, 0x4f),
        (0xffce, 0x4f),
        (0xffcc, 0x4f),
        (0xffca, 0x4f),
        (0xffc9, 0x4f),
        (0xffc6, 0x4f),
        (0xffc4, 0x4f),
        (0xffc2, 0x4f),
        (0xffc0, 0x4f),
        (0xff22, 0x07),
        (0xffd2, 0x4b),
        (0xffd0, 0x4b),
        (0xffce, 0x4b),
        (0xffcc, 0x4b),
        (0xffca, 0x4b),
        (0xffc9, 0x4b),
        (0xffc6, 0x4b),
        (0xffc4, 0x4b),
        (0xffc2, 0x4b),
        (0xffc0, 0x4b),
        (0xff22, 0x07),
        (0xffd2, 0x0d),
        (0xffd0, 0x0d),
        (0xffce, 0x0d),
        (0xffcc, 0x0d),
        (0xffca, 0x0d),
        (0xffc9, 0x0d),
        (0xffc6, 0x0d),
        (0xffc4, 0x0d),
        (0xffc2, 0x0d),
        (0xffc0, 0x0d),
        (0xff22, 0x07),
        (0xff02, 0xff),
]
