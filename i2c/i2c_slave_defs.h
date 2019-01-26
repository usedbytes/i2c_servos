/*
 * Copyright Brian Starkey 2014 <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __I2C_SLAVE_DEFS__
#define __I2C_SLAVE_DEFS__

/* Set these appropriately for your platform */
#define USI_PORT PORTB
#define USI_DDR DDRB
#define I2C_SDA 0
#define I2C_SCL 2

/* Set these appropriately for your application */
#define I2C_N_REG 7
#define I2C_SLAVE_ADDR 0x40

/*
 * The library supports a write mask for each individual register (bits set are
 * writable) in the i2c_w_mask array. If you don't care about masks for each
 * individual register, you can define a global value to be used for all
 * registers here, saving flash and RAM
#define I2C_GLOBAL_WRITE_MASK 0xFF
 */

/* Define anything else your application wants to know */
#define REG_CONTROL        i2c_reg[0]
#define REG_SERVO_A        i2c_reg[1]
#define REG_SERVO_B        i2c_reg[2]
#define REG_SERVO_A_MIN    i2c_reg[3]
#define REG_SERVO_A_MAX    i2c_reg[4]
#define REG_SERVO_B_MIN    i2c_reg[5]
#define REG_SERVO_B_MAX    i2c_reg[6]

#endif /* __I2C_SLAVE_DEFS__ */
