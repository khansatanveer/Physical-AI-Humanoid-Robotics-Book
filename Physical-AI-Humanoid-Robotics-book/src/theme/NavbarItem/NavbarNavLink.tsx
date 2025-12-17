import React from 'react';
import type { JSX } from 'react';
import { useLocation } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import type { Props } from '@theme/NavbarItem/NavbarNavLink';

export default function NavbarNavLink({
  activeBasePath,
  activeBaseRegex,
  to,
  href,
  label,
  activeClassName = 'navbar__link--active',
  prependBaseUrlToHref,
  ...props
}: Props): JSX.Element {
  const location = useLocation();
  const toUrl = useBaseUrl(to);
  const active = isActiveNavbarLink(location.pathname, to, activeBasePath, activeBaseRegex);

  const navbarLinkClassNames = cn(
    'navbar__link',
    active && activeClassName,
    props.className
  );

  return (
    <Link
      {...props}
      className={navbarLinkClassNames}
      to={toUrl}
      href={prependBaseUrlToHref ? useBaseUrl(href) : href}
      {...(active && { 'aria-current': 'page' })}>
      {label}
    </Link>
  );
}


function cn(...classes: (string | boolean | undefined)[]) {
  return classes.filter(Boolean).join(' ');
}

function isActiveNavbarLink(
  pathname: string,
  to: string | undefined,
  activeBasePath: string | undefined,
  activeBaseRegex: string | undefined
): boolean {
  if (activeBaseRegex) {
    return new RegExp(activeBaseRegex).test(pathname);
  }
  if (activeBasePath) {
    // Check if pathname starts with activeBasePath (considering proper path boundaries)
    const normalizedPathname = pathname.replace(/\/$/, '');
    const normalizedBasePath = activeBasePath.replace(/\/$/, '');
    return normalizedPathname === normalizedBasePath || normalizedPathname.startsWith(normalizedBasePath + '/');
  }
  if (to) {
    // Check if pathname is the same as 'to' or a subpath of 'to'
    const normalizedPathname = pathname.replace(/\/$/, '');
    const normalizedTo = to.replace(/\/$/, '');
    return normalizedPathname === normalizedTo || normalizedPathname.startsWith(normalizedTo + '/');
  }
  return false;
}