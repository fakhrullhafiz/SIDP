import pyotp
import qrcode
from PIL import Image
import io

# ================================
# PERMANENT SECRET KEY
# (Use a fixed key so the MFA always matches)
# ================================
SECRET_KEY = "KRSXG5DSMFZWC3TJ"  # <-- keep this constant

# Generate TOTP object
totp = pyotp.TOTP(SECRET_KEY)

# Create provisioning URI
uri = totp.provisioning_uri(
    name="user@primetech.com",
    issuer_name="Prime Technologies Sdn. Bhd."
)

# Generate QR code in memory
qr_img = qrcode.make(uri)

# Display QR code in a pop-up window
qr_img.show()

print("=== Prime Technologies MFA DEMO SYSTEM ===")
print("Scan the QR code displayed in the pop-up window using Microsoft Authenticator.\n")
print("Your permanent secret key:", SECRET_KEY)

# Simulated password in the database
password_in_db = "iloveprime"

entered_password = input("Enter your password: ")
if entered_password != password_in_db:
    print("❌ Password incorrect. Access denied.")
    exit()

print("Password correct. Enter the 6-digit MFA code from your Authenticator app.")
mfa_code = input("> ")

if totp.verify(mfa_code):
    print("✅ MFA verified. Access granted!")
else:
    print("❌ Invalid MFA code. Access denied.")
