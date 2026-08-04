path = "src/configure.py"
with open(path, "r") as f:
    content = f.read()

old = """                _, _, _, value = struct.unpack_from(fmt, msg.data)
                return value"""

new = """                opcode, reply_endpoint_id, reserved, value = struct.unpack_from(fmt, msg.data)
                if reply_endpoint_id != endpoint_id:
                    continue  # stale reply meant for a different endpoint request
                              # on this same node - not our answer, keep waiting
                return value"""

count = content.count(old)
if count != 1:
    raise SystemExit(f"Expected exactly 1 match, found {count} - aborting, nothing changed.")

content = content.replace(old, new)
with open(path, "w") as f:
    f.write(content)

print("Patched src/configure.py: read_config() now verifies the reply's endpoint ID before accepting it.")
