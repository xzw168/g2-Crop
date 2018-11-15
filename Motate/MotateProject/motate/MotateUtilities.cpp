
#include "MotateUtilities.h"

//#pragma GCC diagnostic push
// We are getting useless warning about the constexpr not being "inlined"
//#pragma GCC diagnostic ignored "-Winline"


namespace Motate {

	// We'll put these in the Private namespace for now, to indicate that they're private.
	namespace Private {

		constexpr float round_lookup_[] = {
			0.5,          // precision 0
			0.05,         // precision 1
			0.005,        // precision 2
			0.0005,       // precision 3
			0.00005,      // precision 4
			0.000005,     // precision 5
			0.0000005,    // precision 6
			0.00000005,   // precision 7
			0.000000005,  // precision 8
			0.0000000005, // precision 9
			0.00000000005 // precision 10
		};

		int c_floattoa(float in, char *buffer, int maxlen, int precision) {
			int length_ = 0;
			char *b_ = buffer;

			if (in < 0.0) {
				*b_++ = '-';
				return c_floattoa(-in, b_, maxlen - 1, precision) + 1;
			}

			// float round_ = 0.5;
			in = in + round_lookup_[precision];

			int int_length_ = 0;

			int integer_part_ = (int)in;

			while (integer_part_ > 0) {
				if (length_++ > maxlen) {
					*buffer = 0;
					return 0;
				}
				int t_ = integer_part_ / 10;
				*b_++ = '0' + (integer_part_ - (t_ * 10));
				integer_part_ = t_;
				int_length_++;
			}
			if (length_ > 0) {
				c_strreverse(buffer, int_length_);
			}
			else {
				*b_++ = '0';
				int_length_++;
			}

			*b_++ = '.';
			length_ = int_length_ + 1;

			float frac_part_ = in;
			frac_part_ -= (int)frac_part_;
			while (precision-- > 0) {
				if (length_++ > maxlen) {
					*buffer = 0;
					return 0;
				}
				frac_part_ *= 10.0;
				// if (precision==0) {
				//     t_ += 0.5;
				// }
				*b_++ = ('0' + (int)frac_part_);
				frac_part_ -= (int)frac_part_;
			}

			// reduce extra characters
			while (*(b_ - 1) == '0' && length_>1) {
				b_--; *b_ = 0;
				length_--;
			}

			if (*(b_ - 1) == '.') {
				b_--; *b_ = 0;
				length_--;
			}

			return length_;
		}

	} // namespace Private


	int strlen(const char *p) {
		return Private::c_strlen(p);
	}

	int streq(const char *p, const char *q, const size_t n) {
		return
			(
			(!n || !p || !q)
				? 1
				: (
				(*p != *q)
					? 0
					: (
					(!*p || !*q)
						? 1
						: streq(p + 1, q + 1, n - 1)
						)
					)
				);
	}

	float atof(char *&buffer) {
		return Private::c_atof(buffer);
	}

	int strncpy(char * const t, const char *f, int max_len_) {
		return Private::c_strcpy(t, f, max_len_);
	}


} // namespace Motate

//#pragma GCC diagnostic pop
