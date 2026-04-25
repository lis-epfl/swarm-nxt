document.addEventListener("DOMContentLoaded", () => {
  document.querySelectorAll("pre > code").forEach((codeBlock) => {
    const button = document.createElement("button");
    button.className = "copy-icon";
    button.setAttribute("aria-label", "Copy code");
    button.setAttribute("title", "Copy");

    button.innerHTML = `
      <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="#666" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="clipboard-icon">
        <rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect>
        <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path>
      </svg>
    `;

    button.addEventListener("click", () => {
  navigator.clipboard.writeText(codeBlock.innerText).then(() => {
    // Animate checkmark in grey
    button.innerHTML = `
      <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="#666" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="check-icon">
        <polyline points="20 6 9 17 4 12"></polyline>
      </svg>
    `;
    setTimeout(() => {
      button.innerHTML = `
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="#666" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="clipboard-icon">
          <rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect>
          <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path>
        </svg>
      `;
    }, 2000);
  });
});


    const pre = codeBlock.parentNode;
    pre.classList.add("code-block");
    pre.appendChild(button);
  });
});

