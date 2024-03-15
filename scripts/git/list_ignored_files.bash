if [[ "$OSTYPE" == "win32" ]]; then
        git ls-files -v | findstr "^S"
else
        git ls-files -v | grep "^S"
fi