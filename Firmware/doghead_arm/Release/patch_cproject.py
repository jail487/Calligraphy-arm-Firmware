import xml.etree.ElementTree as ET

tree = ET.parse('/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/.cproject')
root = tree.getroot()

debug_config = None
release_config = None

for config in root.findall('.//cconfiguration'):
    if 'Debug' in config.get('id'):
        debug_config = config
    elif 'Release' in config.get('id'):
        release_config = config

if debug_config and release_config:
    # 1. Update Include Paths
    debug_toolchain = debug_config.find('.//toolChain')
    release_toolchain = release_config.find('.//toolChain')
    
    if debug_toolchain and release_toolchain:
        debug_compiler = debug_toolchain.find('.//tool[@name="MCU GCC Compiler"]')
        release_compiler = release_toolchain.find('.//tool[@name="MCU GCC Compiler"]')
        
        if debug_compiler and release_compiler:
            debug_includes = debug_compiler.find('.//option[@name="Include paths"]')
            release_includes = release_compiler.find('.//option[@name="Include paths"]')
            if debug_includes is not None and release_includes is not None:
                release_includes.clear()
                for child in debug_includes:
                    release_includes.append(child)

        # 2. Update Linker Libraries and Paths
        debug_linker = debug_toolchain.find('.//tool[@name="MCU GCC Linker"]')
        release_linker = release_toolchain.find('.//tool[@name="MCU GCC Linker"]')
        if debug_linker and release_linker:
            for opt_name in ["Libraries", "Library search path"]:
                d_opt = debug_linker.find(f'.//option[@name="{opt_name}"]')
                r_opt = release_linker.find(f'.//option[@name="{opt_name}"]')
                if d_opt is not None:
                    if r_opt is None:
                        r_opt = ET.SubElement(release_linker, 'option', d_opt.attrib)
                    else:
                        r_opt.clear()
                        for k, v in d_opt.attrib.items():
                            r_opt.set(k, v)
                    for child in d_opt:
                        r_opt.append(child)

    # 3. Update C++ Compiler Include paths
    if debug_toolchain and release_toolchain:
        debug_cpp = debug_toolchain.find('.//tool[@name="MCU G++ Compiler"]')
        release_cpp = release_toolchain.find('.//tool[@name="MCU G++ Compiler"]')
        if debug_cpp and release_cpp:
            debug_incl = debug_cpp.find('.//option[@name="Include paths"]')
            release_incl = release_cpp.find('.//option[@name="Include paths"]')
            if debug_incl is not None and release_incl is not None:
                release_incl.clear()
                for child in debug_incl:
                    release_incl.append(child)

    # 4. Update Source Entries
    debug_source = debug_config.find('.//sourceEntries')
    release_source = release_config.find('.//sourceEntries')
    if debug_source is not None and release_source is not None:
        release_source.clear()
        for child in debug_source:
            release_source.append(child)

# Remove -fcyclomatic-complexity everywhere
for option in root.findall('.//option'):
    if option.get('value') == '-fcyclomatic-complexity':
        option.set('value', '')
    if option.get('name') == 'Cyclomatic complexity':
        option.set('value', 'false')

tree.write('/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/.cproject', encoding='utf-8', xml_declaration=True)
print("Patch completed.")
